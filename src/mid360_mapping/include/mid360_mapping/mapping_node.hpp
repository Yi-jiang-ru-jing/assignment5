#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdint>
#include <memory>
#include <vector> 
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <tf2_ros/static_transform_broadcaster.h>


namespace mid360_mapping
{
class MappingNode : public rclcpp::Node
{
public:
  MappingNode();
  
private:
  void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  void on_custom_cloud(livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message);
  void on_imu(sensor_msgs::msg::Imu::ConstSharedPtr message);
  void on_save_map(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void on_laser_map(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  void on_registered_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  void on_odometry(nav_msgs::msg::Odometry::ConstSharedPtr message);
  void on_path(nav_msgs::msg::Path::ConstSharedPtr message);
  
  std::uint64_t cloud_count_{0};
  std::uint64_t imu_count_{0};
  // 订阅者
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr laser_map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr registered_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  
  // 发布者
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr fastlio_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_points_pub_;
  
  // 成员变量
  std::string map_frame_;
  std::string body_frame_;
  std::string map_output_path_;
  std::mutex map_mutex_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_;
  rclcpp::Time lasted_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
   
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::vector<Eigen::Vector3d> acc_buffer_;
  Eigen::Matrix3d R_;
  bool R_ready_ = false;
};
}  // namespace mid360_mapping
