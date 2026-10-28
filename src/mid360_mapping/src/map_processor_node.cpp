#include <chrono>
#include <cstdint>
#include <exception>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <limits>
#include <algorithm>
#include <fstream>
#include <filesystem>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <mutex>
#include <queue>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class MapProcessorNode : public rclcpp::Node
{
public:
  MapProcessorNode() : Node("map_processor_node")
  {
    const auto input = declare_parameter<std::string>("input_topic", "/mapping/map");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    output_directory_ = declare_parameter<std::string>("output_directory", "");
    voxel_size_ = declare_parameter<double>("voxel_size", 0.05);
    grid_resolution_ = declare_parameter<double>("grid_resolution", 0.10);
    // These are starting values to tune, not measured ground/obstacle thresholds.
    obstacle_height_min_ = declare_parameter<double>("obstacle_height_min", 0.15);
    obstacle_height_max_ = declare_parameter<double>("obstacle_height_max", 6.0);
    ground_depth_min_ = declare_parameter<double>("ground_depth_min", -0.5);
    max_height_cut_ = declare_parameter<double>("max_height_cut", 8.0);
    // 去噪参数
    mean_k_ = declare_parameter<int>("sor_mean_k", 20);
    stddev_mul_thresh_ = declare_parameter<double>("sor_stddev_mul_thresh", 1.0);
    min_obstacle_points_ = declare_parameter<int>("min_obstacle_points", 1);
    min_component_size_ = declare_parameter<int>("min_component_size", 5);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    if (input.empty() || !std::isfinite(voxel_size_) || voxel_size_ <= 0 ||
      !std::isfinite(grid_resolution_) || grid_resolution_ <= 0 || !std::isfinite(obstacle_height_min_) ||
      !std::isfinite(obstacle_height_max_) || obstacle_height_max_ <= obstacle_height_min_)
    {
      throw std::invalid_argument("Invalid map-processing startup parameters");
    }

    // 订阅
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        ++received_;
        // (void)msg;
        // TODO: Process the accumulated map without duplicating complete-map snapshots.
        // TODO: Remove invalid/outlier points and downsample; handle ground/obstacle geometry.
        // TODO: Publish a cleaned cloud and a grid with a documented origin and occupancy policy.
        pcl::PointCloud<pcl::PointXYZI> temp_in, temp_filtered;
        auto out_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::fromROSMsg(*msg, temp_in);
        std::vector<int> nan_indices;
        pcl::removeNaNFromPointCloud(temp_in, temp_filtered, nan_indices);
        voxel_filter_.setInputCloud(temp_filtered.makeShared());
        voxel_filter_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel_filter_.filter(temp_filtered);
        remove_outlier_filter_.setInputCloud(temp_filtered.makeShared());
        remove_outlier_filter_.setMeanK(mean_k_);
        remove_outlier_filter_.setStddevMulThresh(stddev_mul_thresh_);
        remove_outlier_filter_.filter(temp_filtered);
        
        bool tf_ok = true;
        // 点云坐标转换
        if (msg->header.frame_id != map_frame_) {
          try {
            // 获取 map 到 camera_init 的变换
            geometry_msgs::msg::TransformStamped tf_msg = tf_buffer_->lookupTransform(
                map_frame_, msg->header.frame_id, tf2::TimePointZero);
            Eigen::Affine3d transform = tf2::transformToEigen(tf_msg);
            // 点云旋转
            pcl::transformPointCloud(temp_filtered, temp_filtered, transform.cast<float>());
          }
          catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "Could not transform %s to %s: %s", 
                        msg->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
            tf_ok = false;
          }
        }

        if (!tf_ok) return;

        // 裁掉天花板
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(temp_filtered.makeShared());
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-std::numeric_limits<float>::max(), max_height_cut_);
        pass.filter(temp_filtered);

        pcl::toROSMsg(temp_filtered, *out_msg);
        out_msg->header.frame_id = map_frame_;
        out_msg->header.stamp = msg->header.stamp;
        cleaned_pub_->publish(std::move(out_msg));

        // 格栅化
        double x_min = std::numeric_limits<double>::max();
        double x_max = std::numeric_limits<double>::lowest();
        double y_min = std::numeric_limits<double>::max();
        double y_max = std::numeric_limits<double>::lowest();
        for (const auto& p : temp_filtered) {
          if (p.z > obstacle_height_max_) continue;
          x_min = std::min(x_min, static_cast<double>(p.x));
          x_max = std::max(x_max, static_cast<double>(p.x));
          y_min = std::min(y_min, static_cast<double>(p.y));
          y_max = std::max(y_max, static_cast<double>(p.y));
        }

        // 空高度带保护
        if (x_min > x_max || y_min > y_max) return;
        double origin_x = floor(x_min / grid_resolution_) * grid_resolution_;
        double origin_y = floor(y_min / grid_resolution_) * grid_resolution_;
        int width = (int)(ceil((x_max - origin_x) / grid_resolution_));
        int height = (int)(ceil((y_max - origin_y) / grid_resolution_));
        std::lock_guard<std::mutex> lock(mutex_);
        filtered_ = temp_filtered;
        grid_.data.assign(width * height, -1);
        grid_.header.frame_id = map_frame_;
        grid_.header.stamp = msg->header.stamp;
        
        // 障碍物判定阈值
        std::vector<int> obstacle_counts(width * height, 0);
        // 标记障碍物区域
        for(const auto& p : filtered_){
          if(p.z < obstacle_height_min_ || p.z > obstacle_height_max_){
            continue;
          }
          int ix = static_cast<int>(floor((p.x - origin_x) / grid_resolution_));
          int iy = static_cast<int>(floor((p.y - origin_y) / grid_resolution_));
                    if(ix >= 0 && ix < width && iy >= 0 && iy < height){
            int idx = iy * width + ix;
            obstacle_counts[idx]++;
            // 达到阈值
            if (obstacle_counts[idx] >= min_obstacle_points_) {
              grid_.data[idx] = 100;
            }
          }
        }

        // 标记空闲区域
        for (const auto& p : filtered_) {
          if (p.z >= obstacle_height_min_ || p.z < ground_depth_min_) continue;
          int ix = static_cast<int>(floor((p.x - origin_x) / grid_resolution_));
          int iy = static_cast<int>(floor((p.y - origin_y) / grid_resolution_));
          if(ix >= 0 && ix < width && iy >= 0 && iy < height){
            // 非障碍物，0
            if (grid_.data[iy * width + ix] != 100) {
              grid_.data[iy * width + ix] = 0; 
            }
          }
        }

        // 连通域过滤
        std::vector<char> visited(width * height, 0); // 0表示未访问
        for (int y = 0; y < height; ++y) {
          for (int x = 0; x < width; ++x) {
            int idx = y * width + x;
            // 障碍物且未访问，启用
            if (grid_.data[idx] == 100 && visited[idx] == 0) {
              std::queue<int> q;
              std::vector<int> current_component;
              q.push(idx);
              visited[idx] = 1;
              current_component.push_back(idx);
              // BFS 泛洪
              while (!q.empty()) {
                int curr = q.front();
                q.pop();
                int cx = curr % width;
                int cy = curr / width;
                // 检查 8 邻域
                for (int dy = -1; dy <= 1; ++dy) {
                  for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = cx + dx;
                    int ny = cy + dy;
                    // 边界越界保护
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                      int n_idx = ny * width + nx;
                      if (grid_.data[n_idx] == 100 && visited[n_idx] == 0) {
                        q.push(n_idx);
                        visited[n_idx] = 1;
                        current_component.push_back(n_idx);
                      }
                    }
                  }
                }
              }
              // 小于 min_component_size_，标为-1
              if (current_component.size() < static_cast<size_t>(min_component_size_)) {
                for (int comp_idx : current_component) {
                  grid_.data[comp_idx] = -1;
                }
              }
            }
          }
        }

        // 发布
        grid_.info.origin.position.x = origin_x;
        grid_.info.origin.position.y = origin_y;
        grid_.info.origin.position.z = 0.0;
        // rviz位姿解析
        grid_.info.origin.orientation.x = 0.0;
        grid_.info.origin.orientation.y = 0.0;
        grid_.info.origin.orientation.z = 0.0;
        grid_.info.origin.orientation.w = 1.0; 
        
        grid_.info.resolution = grid_resolution_;
        grid_.info.width = width;
        grid_.info.height = height;
        grid_pub_->publish(grid_);
      });
    cleaned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "mapping/map_filtered", rclcpp::QoS(20));
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "mapping/map_2d", rclcpp::QoS(1).reliable().transient_local());
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "mapping/save_artifacts",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        // TODO: Save cleaned PCD and a corresponding PGM/YAML pair; report real I/O failures.
        RCLCPP_INFO(get_logger(), "save_artifacts called");
        pcl::PointCloud<pcl::PointXYZI> filtered_copy;
        nav_msgs::msg::OccupancyGrid grid_copy;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          filtered_copy = filtered_;
          grid_copy = grid_;
        }
        if (grid_copy.info.width == 0 || grid_copy.info.height == 0) {
          response->success = false;
          response->message = "Cannot save artifacts: grid is empty (no valid points received yet)";
          return;
        }
        // 导出
        if (output_directory_.empty()) {
          response->success = false;
          response->message = "output_directory parameter is empty, cannot save artifacts";
          return;
        }
        try {
          // 导出PCD
          std::string pcd_path = output_directory_ + "/cleaned.pcd";
          if (pcl::io::savePCDFileBinary(pcd_path, filtered_copy) != 0) {
            throw std::runtime_error("Failed to save PCD file to " + pcd_path);
          }
          // 导出PGM YAML
          if (grid_copy.info.width > 0 && grid_copy.info.height > 0) {
            std::string pgm_path = output_directory_ + "/map.pgm";
            std::string yaml_path = output_directory_ + "/map.yaml";
            // PGM
            std::ofstream pgm_file(pgm_path, std::ios::binary);
            if (!pgm_file) throw std::runtime_error("Failed to open PGM file for writing.");
            pgm_file << "P5\n" << grid_copy.info.width << " " << grid_copy.info.height << "\n255\n";
            int width = static_cast<int>(grid_copy.info.width);
            int height = static_cast<int>(grid_copy.info.height);
            for (int iy = height - 1; iy >= 0; --iy) {
              for (int ix = 0; ix < width; ++ix) {
                int8_t val = grid_copy.data[iy * width + ix];
                unsigned char pixel;
                if (val == 100)
                  pixel = 0;
                else if (val == 0)
                  pixel = 254;
                else
                  pixel = 205;
                pgm_file.write(reinterpret_cast<const char*>(&pixel), 1);
              }
            }
            pgm_file.close();
            // 导出YAML
            std::ofstream yaml_file(yaml_path);
            if (!yaml_file) throw std::runtime_error("Failed to open YAML file for writing.");
            
            yaml_file << "image: map.pgm\n";
            yaml_file << "resolution: " << grid_.info.resolution << "\n";
            yaml_file << "origin: [" 
                      << grid_.info.origin.position.x << ", " 
                      << grid_.info.origin.position.y << ", 0.0]\n";
            yaml_file << "negate: 0\n";
            yaml_file << "occupied_thresh: 0.65\n";
            yaml_file << "free_thresh: 0.196\n";
            yaml_file.close();
          }
          else
            RCLCPP_WARN(get_logger(), "Grid is empty, skipped PGM/YAML export.");
          response->success = true;
          response->message = "Successfully saved artifacts to " + output_directory_;
        }
        catch (const std::exception& e) {
          response->success = false;
          response->message = "Error occurred while saving artifacts: " + std::string(e.what());
        }
      });
    timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
      RCLCPP_INFO(get_logger(), "Received %llu map snapshots, processed and published.",
        static_cast<unsigned long long>(received_));
    });
  }

private:
  std::uint64_t received_{0};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cleaned_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string map_frame_;
  std::string output_directory_;
  int mean_k_;
  int min_obstacle_points_;
  int min_component_size_;
  double stddev_mul_thresh_;
  double voxel_size_;
  double grid_resolution_;
  double obstacle_height_min_;
  double obstacle_height_max_;
  double ground_depth_min_;
  double max_height_cut_;
  double robot_radius_;
  bool mark_path_free_;
  pcl::PointCloud<pcl::PointXYZI> filtered_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> remove_outlier_filter_;
  std::mutex mutex_ ;
  nav_msgs::msg::OccupancyGrid grid_;
  std::vector<geometry_msgs::msg::Point> trajectory_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {rclcpp::spin(std::make_shared<MapProcessorNode>());}
  catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("map_processor_node"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
