#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

namespace fs = std::filesystem;

class PointCloudToGridMap : public rclcpp::Node
{
public:
  PointCloudToGridMap() : Node("pointcloud_to_gridmap_node"), valid_(true)
  {
    // 声明参数
    declare_parameter<std::string>("pointcloud_file", "map.stcm");
    declare_parameter<std::string>("output_dir", "./output_map");
    declare_parameter<double>("map_resolution", 0.05);
    declare_parameter<double>("z_min", -0.5);
    declare_parameter<double>("z_max", 0.5);
    declare_parameter<int>("occupied_threshold", 1);
    declare_parameter<int>("unknown_fill", 205);
    declare_parameter<int>("free_fill", 254);
    declare_parameter<int>("occupied_fill", 0);

    // 获取参数
    pointcloud_file_ = get_parameter("pointcloud_file").as_string();
    output_dir_ = get_parameter("output_dir").as_string();
    resolution_ = get_parameter("map_resolution").as_double();
    z_min_ = get_parameter("z_min").as_double();
    z_max_ = get_parameter("z_max").as_double();
    occupied_threshold_ = get_parameter("occupied_threshold").as_int();
    unknown_fill_ = get_parameter("unknown_fill").as_int();
    free_fill_ = get_parameter("free_fill").as_int();
    occupied_fill_ = get_parameter("occupied_fill").as_int();

    RCLCPP_INFO(this->get_logger(), "Loading point cloud from: %s", pointcloud_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "Z range: [%f, %f]", z_min_, z_max_);

    // 创建输出目录
    if (!fs::exists(output_dir_)) {
      fs::create_directories(output_dir_);
    }

    // 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!loadPointCloud(pointcloud_file_, cloud)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load point cloud file. Node will exit.");
      valid_ = false;
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu points.", cloud->size());

    // 根据 Z 范围过滤
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filterByZ(cloud, filtered_cloud, z_min_, z_max_);
    RCLCPP_INFO(this->get_logger(), "After Z-filter: %zu points.", filtered_cloud->size());

    if (filtered_cloud->empty()) {
      RCLCPP_ERROR(this->get_logger(), "No points left after Z-filtering.");
      valid_ = false;
      return;
    }

    // 生成占用栅格地图
    cv::Mat occupancy_grid;
    double origin_x, origin_y;
    generateOccupancyGrid(filtered_cloud, occupancy_grid, origin_x, origin_y);

    // 保存地图
    std::string pgm_path = output_dir_ + "/map.pgm";
    std::string yaml_path = output_dir_ + "/map.yaml";
    savePGM(occupancy_grid, pgm_path);
    saveYAML(occupancy_grid, pgm_path, origin_x, origin_y, yaml_path);

    RCLCPP_INFO(this->get_logger(), "Map saved to %s", output_dir_.c_str());
  }

  bool is_valid() const { return valid_; }

private:
  std::string pointcloud_file_;
  std::string output_dir_;
  double resolution_;
  double z_min_, z_max_;
  int occupied_threshold_;
  int unknown_fill_;
  int free_fill_;
  int occupied_fill_;
  bool valid_;

  bool loadPointCloud(const std::string& filename,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    if (!fs::exists(filename)) {
      RCLCPP_ERROR(this->get_logger(), "File does not exist: %s", filename.c_str());
      return false;
    }

    std::string ext = fs::path(filename).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    if (ext == ".pcd") {
      return (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == 0);
    } else if (ext == ".ply") {
      return (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud) == 0);
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown extension %s, trying as PCD format first.", ext.c_str());
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == 0)
        return true;
      RCLCPP_WARN(this->get_logger(), "PCD failed, trying PLY format.");
      if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud) == 0)
        return true;
      RCLCPP_ERROR(this->get_logger(), "Failed to load as PCD or PLY.");
      return false;
    }
  }

  void filterByZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
                 double z_min, double z_max)
  {
    output->clear();
    for (const auto& point : input->points) {
      if (point.z >= z_min && point.z <= z_max) {
        output->push_back(point);
      }
    }
    output->width = output->size();
    output->height = 1;
    output->is_dense = true;
  }

  void generateOccupancyGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                             cv::Mat& grid,
                             double& origin_x,
                             double& origin_y)
  {
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = -std::numeric_limits<double>::max();
    double max_y = -std::numeric_limits<double>::max();

    for (const auto& pt : cloud->points) {
      min_x = std::min(min_x, static_cast<double>(pt.x));
      min_y = std::min(min_y, static_cast<double>(pt.y));
      max_x = std::max(max_x, static_cast<double>(pt.x));
      max_y = std::max(max_y, static_cast<double>(pt.y));
    }

    double margin = resolution_;
    min_x -= margin;
    min_y -= margin;
    max_x += margin;
    max_y += margin;

    origin_x = min_x;
    origin_y = min_y;

    int width = static_cast<int>(std::ceil((max_x - min_x) / resolution_));
    int height = static_cast<int>(std::ceil((max_y - min_y) / resolution_));
    if (width <= 0 || height <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid grid dimensions: %dx%d", width, height);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Grid size: %d x %d (width x height)", width, height);
    grid = cv::Mat(height, width, CV_8UC1, cv::Scalar(unknown_fill_));

    std::vector<std::vector<int>> count(height, std::vector<int>(width, 0));
    for (const auto& pt : cloud->points) {
      int col = static_cast<int>((pt.x - origin_x) / resolution_);
      int row = static_cast<int>((pt.y - origin_y) / resolution_);
      if (col >= 0 && col < width && row >= 0 && row < height) {
        count[row][col]++;
      }
    }

    for (int r = 0; r < height; ++r) {
      for (int c = 0; c < width; ++c) {
        if (count[r][c] >= occupied_threshold_) {
          grid.at<uchar>(r, c) = static_cast<uchar>(occupied_fill_);
        } else if (count[r][c] > 0) {
          grid.at<uchar>(r, c) = static_cast<uchar>(free_fill_);
        }
      }
    }
  }

  void savePGM(const cv::Mat& grid, const std::string& filename)
  {
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_PXM_BINARY);
    cv::imwrite(filename, grid, compression_params);
    RCLCPP_INFO(this->get_logger(), "Saved PGM: %s", filename.c_str());
  }

  void saveYAML(const cv::Mat& /*grid*/, const std::string& pgm_path,
                double origin_x, double origin_y, const std::string& filename)
  {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "image" << YAML::Value << fs::path(pgm_path).filename().string();
    out << YAML::Key << "resolution" << YAML::Value << resolution_;
    out << YAML::Key << "origin" << YAML::Value << YAML::Flow
        << YAML::BeginSeq << origin_x << origin_y << 0.0 << YAML::EndSeq;
    out << YAML::Key << "negate" << YAML::Value << 0;
    out << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
    out << YAML::Key << "free_thresh" << YAML::Value << 0.196;
    out << YAML::EndMap;

    std::ofstream fout(filename);
    fout << out.c_str();
    fout.close();
    RCLCPP_INFO(this->get_logger(), "Saved YAML: %s", filename.c_str());
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudToGridMap>();
  if (node->is_valid()) {
    rclcpp::spin(node);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Node initialization failed, exiting.");
  }
  rclcpp::shutdown();
  return 0;
}