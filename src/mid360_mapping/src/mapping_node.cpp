#include "mid360_mapping/mapping_node.hpp"
#include <chrono>
#include <stdexcept>
#include <string>
#include <cstring>
#include <cmath>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <pcl/filters/passthrough.h>


namespace mid360_mapping
{
MappingNode::MappingNode() : Node("mapping_node"), map_cloud_(new pcl::PointCloud<pcl::PointXYZI>())
{
  rcl_interfaces::msg::ParameterDescriptor startup;
  startup.read_only = true;
  startup.description = "Startup setting; restart the node after changing configuration";
  const auto input_type = declare_parameter<std::string>("input_type", "pointcloud2", startup);
  const auto lidar_topic = declare_parameter<std::string>("lidar_topic", "/livox/lidar", startup);
  const auto imu_topic = declare_parameter<std::string>("imu_topic", "/livox/imu", startup);
  
  map_frame_ = declare_parameter<std::string>("map_frame", "map", startup);
  body_frame_ = declare_parameter<std::string>("body_frame", "body", startup);
  map_output_path_ = declare_parameter<std::string>("map_output_path", "", startup);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
  // FAST_LIO话题相关
  laser_map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/Laser_map", rclcpp::SensorDataQoS().keep_last(1),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {on_laser_map(msg);});
  
  registered_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/cloud_registered", rclcpp::SensorDataQoS().keep_last(10),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {on_registered_cloud(msg);});
  
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/Odometry", 10,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {on_odometry(msg);});
  
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/path", 10,
    [this](nav_msgs::msg::Path::ConstSharedPtr msg) {on_path(msg);});
  
  // 话题校验
  if (lidar_topic.empty() || imu_topic.empty()) {
    throw std::invalid_argument("lidar_topic and imu_topic must not be empty");
  }

  // 订阅话题
  if (input_type == "pointcloud2") {
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic, rclcpp::SensorDataQoS().keep_last(10),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {on_cloud(msg);});
  }
  else if (input_type == "livox_custom") {
    // A generic subscription avoids a compile-time Livox SDK dependency in the scaffold.
    // The livox_ros_driver2 message type support must still be installed and sourced.
    custom_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      lidar_topic, rclcpp::SensorDataQoS().keep_last(10),
      [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {on_custom_cloud(msg);});
  }
  else {
    throw std::invalid_argument("input_type must be pointcloud2 or livox_custom");
  }
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS().keep_last(1000),
    [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {on_imu(msg);});

  // These are output contracts, not an implemented mapping pipeline.
  // 发布者初始化
  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("mapping/map", rclcpp::SensorDataQoS());
  registered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "mapping/cloud_registered", rclcpp::SensorDataQoS());
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("mapping/odom", 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("mapping/path", 10);
  fastlio_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>("mapping/custom_cloud", rclcpp::QoS(20));
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("mapping/imu", rclcpp::QoS(20));
  lidar_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("mapping/lidar_points", rclcpp::QoS(20));
  
  // 服务与定时器
  save_service_ = create_service<std_srvs::srv::Trigger>(
    "mapping/save_map",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      // TODO: OK; Save the accumulated map to map_output_path and report the actual result.
      on_save_map(request, response);
    });
  status_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
    size_t map_size = map_cloud_ ? map_cloud_->size() : 0;
    RCLCPP_INFO(
      get_logger(), "Received clouds=%llu imu=%llu; map size=%zu points",
      static_cast<unsigned long long>(cloud_count_),
      static_cast<unsigned long long>(imu_count_),
      map_size);
  });
}

// 接收地图并转发
void MappingNode::on_laser_map(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    pcl::fromROSMsg(*message, *map_cloud_);
  }
  map_pub_->publish(*message);
}

// 接收注册点云并转发
void MappingNode::on_registered_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  registered_pub_->publish(*message);
}

// 接收里程计并转发，发布TF
void MappingNode::on_odometry(nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  odom_pub_->publish(*message);
}

// 接收路径并转发
void MappingNode::on_path(nav_msgs::msg::Path::ConstSharedPtr message)
{ 
  path_pub_->publish(*message);
}


void MappingNode::on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  ++cloud_count_;
  //(void)message;
  // TODO: Read actual point fields and units, including per-point acquisition times.
  // TODO: Connect to your chosen mapping backend; preserve sensor timestamps and frames.
  std::string fields_list = pcl::getFieldsList(*message);

  int time_idx = pcl::getFieldIndex(*message, "timestamp");
  if (time_idx == -1) time_idx = pcl::getFieldIndex(*message, "time");
  if (time_idx == -1) time_idx = pcl::getFieldIndex(*message, "t");

  // 获取 x, y, z 的索引
  int x_idx = pcl::getFieldIndex(*message, "x");
  int y_idx = pcl::getFieldIndex(*message, "y");
  int z_idx = pcl::getFieldIndex(*message, "z");

  uint32_t num_points = message->width * message->height;
  if (num_points == 0) return;
  
  const uint8_t* data_ptr = message->data.data();
  rclcpp::Time base_stamp(message->header.stamp);
  std::vector<std::pair<rclcpp::Time, Eigen::Vector3d>> points;
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
  points.reserve(num_points);
  pcl_cloud.reserve(num_points);
  //sensor_msgs::PointCloud2ConstIterator<float> iter_x(*message, "x");
  //sensor_msgs::PointCloud2ConstIterator<float> iter_y(*message, "y");
  //sensor_msgs::PointCloud2ConstIterator<float> iter_z(*message, "z");
  //sensor_msgs::PointCloud2ConstIterator<double> iter_time(*message, "timestamp");
  //for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_time) {
    //int64_t t = static_cast<int64_t>(*iter_time);
    //const rclcpp::Time stamp = base_stamp + rclcpp::Duration(std::chrono::nanoseconds(t));
    //points.emplace_back(stamp, Eigen::Vector3d(*iter_x, *iter_y, *iter_z));
  //} 
  //(void)points;

  if (time_idx == -1) {
    std::string fields_list = pcl::getFieldsList(*message);
    RCLCPP_WARN_ONCE(get_logger(), 
      "PointCloud2 has no time field. Motion distortion compensation degraded. Available fields: %s", 
      fields_list.c_str());

    for (uint32_t i = 0; i < num_points; ++i) {
      const uint8_t* pt_ptr = data_ptr + i * message->point_step;
      pcl::PointXYZI pt;
      std::memcpy(&pt.x, pt_ptr + message->fields[x_idx].offset, sizeof(float));
      std::memcpy(&pt.y, pt_ptr + message->fields[y_idx].offset, sizeof(float));
      std::memcpy(&pt.z, pt_ptr + message->fields[z_idx].offset, sizeof(float));
      pt.intensity = 0.0f;
      pcl_cloud.push_back(pt);
    }
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(pcl_cloud, out_msg);
    out_msg.header = message->header;
    lidar_points_pub_->publish(out_msg);
    return;
  }

  // 获取时间字段的 offset 和 datatype
  int time_offset = message->fields[time_idx].offset;
  uint8_t time_datatype = message->fields[time_idx].datatype;

  // 单位判定
  double first_time = 0.0;
  if (time_datatype == sensor_msgs::msg::PointField::FLOAT64) {
    double t_val;
    std::memcpy(&t_val, data_ptr + time_offset, sizeof(double));
    first_time = t_val;
  }
  else if (time_datatype == sensor_msgs::msg::PointField::FLOAT32) {
    float t_val;
    std::memcpy(&t_val, data_ptr + time_offset, sizeof(float));
    first_time = static_cast<double>(t_val);
  }
  else {
    RCLCPP_ERROR_ONCE(get_logger(), "Unsupported time datatype. Dropping.");
    return;
  }

  double time_scale = 1.0; // s
  double abs_t = std::abs(first_time);
  if (abs_t < 1.0) time_scale = 1.0;
  else if (abs_t < 1e3) time_scale = 1e-3; // ms
  else if (abs_t < 1e6) time_scale = 1e-6; // us
  else time_scale = 1e-9; // ns


  RCLCPP_INFO_ONCE(get_logger(), "Detected time field '%s', scale %.1e to seconds.", 
                   message->fields[time_idx].name.c_str(), time_scale);

  // 寻址
  for (uint32_t i = 0; i < num_points; ++i) {
    const uint8_t* pt_ptr = data_ptr + i * message->point_step;
    // XYZ
    pcl::PointXYZI pt;
    std::memcpy(&pt.x, pt_ptr + message->fields[x_idx].offset, sizeof(float));
    std::memcpy(&pt.y, pt_ptr + message->fields[y_idx].offset, sizeof(float));
    std::memcpy(&pt.z, pt_ptr + message->fields[z_idx].offset, sizeof(float));
    // 时间
    double t_val = 0.0;
    if (time_datatype == sensor_msgs::msg::PointField::FLOAT64) {
      std::memcpy(&t_val, pt_ptr + time_offset, sizeof(double));
    }
    else {
      float tf;
      std::memcpy(&tf, pt_ptr + time_offset, sizeof(float));
      t_val = static_cast<double>(tf);
    }
    pt.intensity = static_cast<float>(t_val * time_scale); 
    pcl_cloud.push_back(pt);
  }

  // 点云发布
  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(pcl_cloud, out_msg);
  out_msg.header = message->header;
  lidar_points_pub_->publish(out_msg);
}

void MappingNode::on_custom_cloud(livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message)
{
  ++cloud_count_;
  //(void)message;
  // TODO: Replace serialized-message reception with a typed Livox input adapter.
  // Declare livox_ros_driver2 in package.xml and CMake when using its C++ message type.
  // TODO: Preserve timebase, offset_time, reflectivity, tag and line as needed by your backend.
  // 校验点数一致性
  if (message->points.size() != message->point_num) {
    RCLCPP_WARN(get_logger(), "Received CustomMsg with inconsistent size: points.size()=%zu, point_num=%u. Dropping.",
                message->points.size(), message->point_num);
    return;
  }
  if(message->point_num == 0) return;
  
  fastlio_pub_->publish(*message);
}


void MappingNode::on_imu(sensor_msgs::msg::Imu::ConstSharedPtr message)
{
  //(void)message;
  ++imu_count_;

  // 时间戳校验
  const rclcpp::Time stamp = message->header.stamp;
  if (stamp.nanoseconds() == 0) {
    RCLCPP_WARN_ONCE(get_logger(), "IMU message has zero timestamp; dropping.");
    return;
  }

  // 严格单调递增检查
  if (lasted_imu_time_.nanoseconds() != 0 && stamp <= lasted_imu_time_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
      "IMU timestamp non-monotonic (current <= last); dropping to avoid FAST-LIO buffer reset.");
    return;
  }
  lasted_imu_time_ = stamp;

  // 分量校验
  const auto& acc = message->linear_acceleration;
  const auto& gyro = message->angular_velocity;
  if (!std::isfinite(acc.x) || !std::isfinite(acc.y) || !std::isfinite(acc.z) ||
      !std::isfinite(gyro.x) || !std::isfinite(gyro.y) || !std::isfinite(gyro.z)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
      "IMU contains NaN or Inf values; dropping to protect EKF state.");
    return;
  }

  // 加速度模长合理性判断
  const double g = 9.80665;
  const double acc_norm = std::sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z) * g;
  if (acc_norm < 6.9 || acc_norm > 12.8) { 
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
      "IMU acceleration norm is anomalous (%.2f). Please verify units (m/s^2 vs g).", acc_norm);
  }
  
  // 检查orientation
  const auto& ori = message->orientation;
  const double ori_norm = std::sqrt(ori.x * ori.x + ori.y * ori.y + ori.z * ori.z + ori.w * ori.w);
  if (ori_norm < 1e-3) {
    RCLCPP_WARN_ONCE(get_logger(), "IMU orientation is invalid (all zeros). FAST-LIO will rely on its own EKF.");
  }
  else if (std::abs(ori_norm - 1.0) > 0.1) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
      "IMU orientation quaternion norm is far from 1.0 (%.2f).", ori_norm);
  }

  auto out_msg = *message;
  out_msg.linear_acceleration.x *= g;
  out_msg.linear_acceleration.y *= g;
  out_msg.linear_acceleration.z *= g;

  // 重力对齐计算
  if (!R_ready_) {
    Eigen::Vector3d acc(out_msg.linear_acceleration.x, out_msg.linear_acceleration.y, out_msg.linear_acceleration.z);
    acc_buffer_.push_back(acc);

    // 收集约 1 秒
    if (acc_buffer_.size() >= 200) {
      Eigen::Vector3d avg_acc = Eigen::Vector3d::Zero();
      for (const auto& a : acc_buffer_) avg_acc += a;
      avg_acc /= acc_buffer_.size();
      
      // 归一化 --u
      Eigen::Vector3d u = avg_acc.normalized();
      Eigen::Vector3d z_axis(0, 0, 1);
      
      // 计算把 u 转到 z_axis 的最小旋转 R
      double cos_theta = u.dot(z_axis);
      if (cos_theta > 0.9999) {
        R_ = Eigen::Matrix3d::Identity();
      }
      else if (cos_theta < -0.9999) {
        R_ = Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
      }
      else {
        Eigen::Vector3d axis = u.cross(z_axis);
        double angle = std::acos(cos_theta);
        R_ = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
      }
      
      R_ready_ = true;
      RCLCPP_INFO(get_logger(), "Calculated gravity alignment R! Tilt angle ~ %.2f deg", std::acos(cos_theta) * 180.0 / M_PI);

      // 发布静态 TF: map -> camera_init
      geometry_msgs::msg::TransformStamped static_tf;
      static_tf.header.stamp = this->get_clock()->now();
      static_tf.header.frame_id = map_frame_;
      static_tf.child_frame_id = "camera_init"; // FAST-LIO 的默认根坐标系
      
      Eigen::Quaterniond q(R_);
      static_tf.transform.rotation.w = q.w();
      static_tf.transform.rotation.x = q.x();
      static_tf.transform.rotation.y = q.y();
      static_tf.transform.rotation.z = q.z();
      static_tf.transform.translation.x = 0;
      static_tf.transform.translation.y = 0;
      static_tf.transform.translation.z = 0;
      
      static_tf_broadcaster_->sendTransform(static_tf);
    }
  }

  imu_pub_->publish(out_msg);
}


void MappingNode::on_save_map(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (map_output_path_.empty()) {
    response->success = false;
    response->message = "map_output_path parameter is empty; cannot save map";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr snapshot(new pcl::PointCloud<pcl::PointXYZI>);
  Eigen::Matrix3d R_copy = Eigen::Matrix3d::Identity(); 
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    *snapshot = *map_cloud_;
    R_copy = R_;
  }
  if (snapshot->empty()) {
    response->success = false;
    response->message = "Map is empty; nothing to save";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }
  // 转至水平坐标系
  if (R_ready_) {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.linear() = R_copy.cast<float>();
    pcl::transformPointCloud(*snapshot, *snapshot, transform);
    RCLCPP_INFO(get_logger(), "Applied gravity alignment rotation to saved PCD.");
  }
  else {
    RCLCPP_WARN(get_logger(), "Rotation matrix R is not ready yet. Saving original (tilted) PCD.");
  }

  
  if (pcl::io::savePCDFileBinary(map_output_path_, *snapshot) == 0) {
    response->success = true;
    response->message = "Map saved successfully to " + map_output_path_;
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }
  else {
    response->success = false;
    response->message = "Failed to save map to " + map_output_path_;
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

}  // namespace mid360_mapping
}
