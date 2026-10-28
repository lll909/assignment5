#include "mid360_mapping/mapping_node.hpp"

#include <chrono>
#include <stdexcept>
#include <string>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace mid360_mapping
{

MappingNode::MappingNode() : Node("mapping_node")
{
  rcl_interfaces::msg::ParameterDescriptor startup;
  startup.read_only = true;
  startup.description =
    "Startup setting; restart the node after changing configuration";

  const auto input_type =
    declare_parameter<std::string>("input_type", "pointcloud2", startup);

  const auto lidar_topic =
    declare_parameter<std::string>("lidar_topic", "/livox/lidar", startup);

  const auto imu_topic =
    declare_parameter<std::string>("imu_topic", "/livox/imu", startup);

  declare_parameter<std::string>("map_frame", "map", startup);
  declare_parameter<std::string>("body_frame", "body", startup);
  declare_parameter<std::string>("map_output_path", "", startup);

  if (lidar_topic.empty() || imu_topic.empty()) {
    throw std::invalid_argument(
      "lidar_topic and imu_topic must not be empty");
  }

  // 订阅 PointCloud2 格式的点云
  if (input_type == "pointcloud2") {
    cloud_sub_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic,
      rclcpp::SensorDataQoS().keep_last(10),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        on_cloud(msg);
      });

  // 订阅 Livox 原始 CustomMsg 格式的点云
  } else if (input_type == "livox_custom") {
    custom_sub_ =
      create_subscription<livox_ros_driver2::msg::CustomMsg>(
      lidar_topic,
      rclcpp::SensorDataQoS().keep_last(10),
      [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {
        on_custom_cloud(msg);
      });

  } else {
    throw std::invalid_argument(
      "input_type must be pointcloud2 or livox_custom");
  }

  // 订阅 IMU，仅统计接收数量
  imu_sub_ =
    create_subscription<sensor_msgs::msg::Imu>(
    imu_topic,
    rclcpp::SensorDataQoS().keep_last(1000),
    [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
      on_imu(msg);
    });

  // 发布当前帧点云
  map_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>(
    "mapping/map", rclcpp::SensorDataQoS());

  registered_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>(
    "mapping/cloud_registered", rclcpp::SensorDataQoS());

  odom_pub_ =
    create_publisher<nav_msgs::msg::Odometry>(
    "mapping/odom", 10);

  path_pub_ =
    create_publisher<nav_msgs::msg::Path>(
    "mapping/path", 10);

  // 保留原有保存服务，当前版本不进行地图累积
  save_service_ =
    create_service<std_srvs::srv::Trigger>(
    "mapping/save_map",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

      if (!global_map_ || global_map_->empty()) {
        response->success = false;
        response->message = "No accumulated map. Save the full map through Point-LIO instead.";
        return;
      }
      std::string path =
        get_parameter("map_output_path").as_string();

      if (path.empty()) {
        path = "maps/map.pcd";
      }

      try {
        if (pcl::io::savePCDFileBinary(path, *global_map_) < 0) {
          throw std::runtime_error("PCD write failed: " + path);
        }
        response->success = true;
        response->message = "Saved to " + path;
      } catch (const std::exception & e) {
        response->success = false;
        response->message = "Failed: " + std::string(e.what());
      }
    });

  status_timer_ =
    create_wall_timer(
    std::chrono::seconds(5),
    [this]() {
      RCLCPP_INFO(
        get_logger(),
        "Point clouds received: %llu; IMU messages received: %llu",
        static_cast<unsigned long long>(cloud_count_),
        static_cast<unsigned long long>(imu_count_));
    });

  RCLCPP_INFO(
    get_logger(),
    "Mapping node started in current-frame point cloud mode");
}


// PointCloud2 输入：只发布当前收到的这一帧
void MappingNode::on_cloud(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  ++cloud_count_;

  // 直接发布当前帧，不累加到 global_map_
  sensor_msgs::msg::PointCloud2 output = *message;

  // 保留输入消息的时间戳和坐标系
  output.header = message->header;

  map_pub_->publish(output);
}


// Livox CustomMsg 输入：转换为 PointCloud2 后发布当前帧
void MappingNode::on_custom_cloud(
  livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
{
  ++cloud_count_;

  pcl::PointCloud<pcl::PointXYZI> current_cloud;
  current_cloud.reserve(msg->points.size());

  for (const auto & pt : msg->points) {
    pcl::PointXYZI p;

    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;
    p.intensity = static_cast<float>(pt.reflectivity);

    current_cloud.push_back(p);
  }

  // 转换当前帧为 ROS 2 PointCloud2 消息
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(current_cloud, output);

  // 保留雷达消息的时间戳和坐标系
  output.header.stamp = msg->header.stamp;
  output.header.frame_id = msg->header.frame_id;

  map_pub_->publish(output);
}


// IMU：当前仅统计接收数量，不执行 SLAM
void MappingNode::on_imu(
  sensor_msgs::msg::Imu::ConstSharedPtr message)
{
  ++imu_count_;
  (void)message;
}

}  // namespace mid360_mapping