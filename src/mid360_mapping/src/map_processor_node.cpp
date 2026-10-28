#include <chrono>
#include <cstdint>
#include <exception>
#include <cmath>
#include <memory>
#include <limits>
#include <vector>
#include <pcl/filters/statistical_outlier_removal.h>
#include <stdexcept>
#include <string>
#include <fstream>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

class MapProcessorNode : public rclcpp::Node
{
public:
  MapProcessorNode() : Node("map_processor_node")
  {
    const auto input = declare_parameter<std::string>("input_topic", "/pcd_full_map"); // 默认改为了 Point-LIO 的话题
    const auto map_frame = declare_parameter<std::string>("map_frame", "camera_init"); // 改为 Point-LIO 的坐标系
    const auto out_dir = declare_parameter<std::string>("output_directory", "./maps");
    voxel_size_ = declare_parameter<double>("voxel_size", 0.10);
    resolution_ = declare_parameter<double>("grid_resolution", 0.10);
    min_height_ = declare_parameter<double>("obstacle_height_min", 0.15);
    max_height_ = declare_parameter<double>("obstacle_height_max", 1.50);

    remove_outliers_ = declare_parameter<bool>("remove_outliers", false);
    outlier_mean_k_ = declare_parameter<int>("outlier_mean_k", 20);
    outlier_stddev_ = declare_parameter<double>("outlier_stddev", 1.0);
    overwrite_existing_ = declare_parameter<bool>("overwrite_existing", false);

    if (map_frame.empty() || outlier_mean_k_ < 2 ||
      !std::isfinite(outlier_stddev_) || outlier_stddev_ <= 0 || input.empty() || !std::isfinite(voxel_size_) || voxel_size_ <= 0 ||
      !std::isfinite(resolution_) || resolution_ <= 0 || !std::isfinite(min_height_) ||
      !std::isfinite(max_height_) || max_height_ <= min_height_)
    {
      throw std::invalid_argument("Invalid map-processing startup parameters");
    }

    output_directory_ = out_dir;
    std::filesystem::create_directories(output_directory_);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input, rclcpp::SensorDataQoS(), [this,map_frame](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        ++received_;
        // 坐标标签不能代替变换；只接收同一坐标系的完整地图。
        if (msg->header.frame_id != map_frame) {
          RCLCPP_ERROR(get_logger(), "Frame mismatch: input=%s, expected=%s; snapshot rejected",
            msg->header.frame_id.c_str(), map_frame.c_str());
          return;
        }
        
        // 1. 转换点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);
         RCLCPP_INFO(
         this->get_logger(),
         "Original cloud: %zu points",
          cloud->size());      
        // 显式剔除 NaN/Inf，避免依赖输入的 is_dense 标记。
        pcl::PointCloud<pcl::PointXYZI>::Ptr finite_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        finite_cloud->reserve(cloud->size());
        for (const auto & pt : cloud->points) {
          if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            auto valid = pt;
            if (!std::isfinite(valid.intensity)) valid.intensity = 0.0f;
            finite_cloud->push_back(valid);
          }
        }
        RCLCPP_INFO(get_logger(), "Finite points: %zu (removed %zu)",
          finite_cloud->size(), cloud->size() - finite_cloud->size());
        if (finite_cloud->empty()) {
          last_filtered_cloud_.reset();
          RCLCPP_WARN(get_logger(), "No finite points; save cache cleared");
          return;
        }
        // 2. 体素降采样（VoxelGrid）
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(finite_cloud);
        voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        voxel.filter(*filtered_cloud);
         RCLCPP_INFO(
        this->get_logger(),
          "After voxel filter: %zu points",
          filtered_cloud->size());
        // 可选统计离群点过滤；放在降采样后以降低计算量。
        if (remove_outliers_ && filtered_cloud->size() > static_cast<std::size_t>(outlier_mean_k_)) {
          pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
          sor.setInputCloud(filtered_cloud);
          sor.setMeanK(outlier_mean_k_);
          sor.setStddevMulThresh(outlier_stddev_);
          pcl::PointCloud<pcl::PointXYZI>::Ptr denoised(new pcl::PointCloud<pcl::PointXYZI>());
          sor.filter(*denoised);
          filtered_cloud = denoised;
          RCLCPP_INFO(get_logger(), "After outlier filter: %zu points", filtered_cloud->size());
        }
        // 3. 高度过滤并投影到二维
        double min_x = std::numeric_limits<double>::infinity();
        double max_x = -min_x, min_y = min_x, max_y = -min_x;
        pcl::PointCloud<pcl::PointXYZI>::Ptr obs_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        for (const auto& pt : filtered_cloud->points) {
          if (pt.z >= min_height_ && pt.z <= max_height_) {
            obs_cloud->push_back(pt);
            if (pt.x < min_x) min_x = pt.x;
            if (pt.x > max_x) max_x = pt.x;
            if (pt.y < min_y) min_y = pt.y;
            if (pt.y > max_y) max_y = pt.y;
          }
        }
          RCLCPP_INFO(
  this->get_logger(),
  "After height filter: %zu points",
  obs_cloud->size());
        // 发布过滤后的三维点云
        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*obs_cloud, out_msg);
        out_msg.header.frame_id = map_frame;
        out_msg.header.stamp = msg->header.stamp;
        cleaned_pub_->publish(out_msg);

        // 4. 生成二维栅格图
        if (obs_cloud->empty()) {
          last_filtered_cloud_.reset();
          RCLCPP_WARN(get_logger(), "Height filter produced an empty cloud; save cache cleared");
          return;
        }

        double width = max_x - min_x;
        double height = max_y - min_y;
        const double cells_x = std::floor(width / resolution_) + 1;
        const double cells_y = std::floor(height / resolution_) + 1;
        // 防止异常坐标/过细分辨率导致溢出或巨大分配。
        constexpr double max_cells = 25000000.0;
        if (!std::isfinite(cells_x) || !std::isfinite(cells_y) ||
          cells_x < 1 || cells_y < 1 || cells_x * cells_y > max_cells) {
          last_filtered_cloud_.reset();
          RCLCPP_ERROR(get_logger(), "Grid too large or invalid (limit: 25 million cells)");
          return;
        }
        int grid_w = static_cast<int>(cells_x);
        int grid_h = static_cast<int>(cells_y);
        
        nav_msgs::msg::OccupancyGrid grid;
        grid.header.frame_id = map_frame;
        grid.header.stamp = msg->header.stamp;
        grid.info.resolution = resolution_;
        grid.info.width = grid_w;
        grid.info.height = grid_h;
        grid.info.origin.position.x = min_x;
        grid.info.origin.position.y = min_y;
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(grid_w * grid_h, -1); // 未经射线验证的区域保持未知

        for (const auto& pt : obs_cloud->points) {
          int x_idx = (pt.x - min_x) / resolution_;
          int y_idx = (pt.y - min_y) / resolution_;
          if (x_idx >= 0 && x_idx < grid_w && y_idx >= 0 && y_idx < grid_h) {
            grid.data[y_idx * grid_w + x_idx] = 100; // 标记为占据
          }
        }
        
        // 发布二维栅格图
        grid_pub_->publish(grid);
        
        // 保存一份最新的过滤点云和栅格供保存服务使用
        last_filtered_cloud_ = obs_cloud;
        last_grid_ = grid;
      });
      
    cleaned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "mapping/map_filtered", rclcpp::SensorDataQoS());
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "mapping/map_2d", rclcpp::QoS(1).reliable().transient_local());
      
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "mapping/save_artifacts",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (last_filtered_cloud_ == nullptr || last_filtered_cloud_->empty()) {
            response->success = false;
            response->message = "No map data available to save.";
            return;
        }//unsigned
        
        try {
            // 已有成果默认不可覆盖；请使用新的输出目录。
            for (const auto & name : {"map.pcd", "map.pgm", "map.yaml"}) {
              if (!overwrite_existing_ && std::filesystem::exists(
                  std::filesystem::path(output_directory_) / name)) {
                throw std::runtime_error("Output exists; choose a new output_directory: " +
                  output_directory_);
              }
            }
            // 保存 PCD
            std::string pcd_path = output_directory_ + "/map.pcd";
            if (pcl::io::savePCDFileBinary(pcd_path, *last_filtered_cloud_) < 0) {
              throw std::runtime_error("PCD write failed: " + pcd_path);
            }
            
            // 保存 PGM
            std::string pgm_path = output_directory_ + "/map.pgm";
            std::ofstream pgm(pgm_path, std::ios::binary);
            pgm.exceptions(std::ios::failbit | std::ios::badbit);
            pgm << "P5\n" << last_grid_.info.width << " " << last_grid_.info.height << "\n255\n";
            for (int y = static_cast<int>(last_grid_.info.height)-1; y >= 0; --y) {
                for (int x = 0; x < static_cast<int>(last_grid_.info.width); ++x) {
                    int index = y * last_grid_.info.width + x;
                    unsigned char val;

if (last_grid_.data[index] == 100) {
    val = 0;       // 障碍物：黑色
} else if (last_grid_.data[index] == 0) {
    val = 254;     // 空闲：白色
} else {
    val = 205;     // 未知：灰色
} // 占据为黑，空闲为白
                    pgm.write(reinterpret_cast<char*>(&val), 1);
                }
            }
            pgm.close();

            // 保存 YAML
            std::string yaml_path = output_directory_ + "/map.yaml";
            std::ofstream yaml(yaml_path);
            yaml.exceptions(std::ios::failbit | std::ios::badbit);
            yaml.precision(std::numeric_limits<double>::max_digits10);
            yaml << "image: map.pgm\n";
            yaml << "resolution: " << last_grid_.info.resolution << "\n";
            yaml << "origin: [" << last_grid_.info.origin.position.x << ", " << last_grid_.info.origin.position.y << ", 0.0]\n";
            yaml << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
            yaml.close();

            response->success = true;
            response->message = "Saved map to " + output_directory_;
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Failed to save: " + std::string(e.what());
        }
      });
      
    timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
      RCLCPP_INFO(get_logger(), "Received %llu map snapshots. 2D/3D map generated.",
        static_cast<unsigned long long>(received_));
    });
  }

private:
  std::uint64_t received_{0};
  double voxel_size_, resolution_, min_height_, max_height_;
  bool remove_outliers_{false}, overwrite_existing_{false};
  int outlier_mean_k_{20};
  double outlier_stddev_{1.0};
  std::string output_directory_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cleaned_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr last_filtered_cloud_{nullptr};
  nav_msgs::msg::OccupancyGrid last_grid_;
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