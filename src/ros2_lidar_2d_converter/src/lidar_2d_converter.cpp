#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <limits>

class Lidar2DConverter : public rclcpp::Node
{
public:
    Lidar2DConverter() : Node("lidar_2d_converter")
    {
        // 声明参数
        this->declare_parameter("output_frame", "base_link");
        this->declare_parameter("min_height", 0.3);
        this->declare_parameter("max_height", 0.757);
        this->declare_parameter("angle_min", -M_PI);
        this->declare_parameter("angle_max", M_PI);
        this->declare_parameter("angle_increment", M_PI/180.0);
        this->declare_parameter("scan_time", 0.1);
        this->declare_parameter("range_min", 0.1);
        this->declare_parameter("range_max", 10.0);
        
        // 获取参数
        output_frame_ = this->get_parameter("output_frame").as_string();
        min_height_ = this->get_parameter("min_height").as_double();
        max_height_ = this->get_parameter("max_height").as_double();
        angle_min_ = this->get_parameter("angle_min").as_double();
        angle_max_ = this->get_parameter("angle_max").as_double();
        angle_increment_ = this->get_parameter("angle_increment").as_double();
        scan_time_ = this->get_parameter("scan_time").as_double();
        range_min_ = this->get_parameter("range_min").as_double();
        range_max_ = this->get_parameter("range_max").as_double();
        
        // 计算扫描点数
        num_readings_ = static_cast<size_t>((angle_max_ - angle_min_) / angle_increment_);
        
        // 创建发布者和订阅者
        laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "rslidar_points", 10, std::bind(&Lidar2DConverter::cloudCallback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "LiDAR 2D Converter node initialized");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        // 转换点云数据
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);
        
        // 初始化激光扫描消息
        auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        
        scan_msg->header = cloud_msg->header;
        scan_msg->header.frame_id = output_frame_;
        scan_msg->angle_min = angle_min_;
        scan_msg->angle_max = angle_max_;
        scan_msg->angle_increment = angle_increment_;
        scan_msg->time_increment = 0.0;
        scan_msg->scan_time = scan_time_;
        scan_msg->range_min = range_min_;
        scan_msg->range_max = range_max_;
        
        // 初始化距离和强度数组
        scan_msg->ranges.resize(num_readings_, std::numeric_limits<float>::infinity());
        scan_msg->intensities.resize(num_readings_, 0.0);
        
        // 处理每个点
        for (const auto& point : cloud.points)
        {
            // 检查高度是否在范围内
            if (point.z < min_height_ || point.z > max_height_)
                continue;
                
            // 计算距离和角度
            float range = std::hypot(point.x, point.y);
            float angle = std::atan2(point.y, point.x);
            
            // 检查距离和角度是否在有效范围内
            if (range < range_min_ || range > range_max_ || angle < angle_min_ || angle > angle_max_)
                continue;
                
            // 计算数组索引
            int index = static_cast<int>((angle - angle_min_) / angle_increment_);
            
            // 确保索引在有效范围内
            if (index >= 0 && index < static_cast<int>(num_readings_))
            {
                // 只保留最近的点
                if (range < scan_msg->ranges[index])
                {
                    scan_msg->ranges[index] = range;
                    scan_msg->intensities[index] = point.z; // 可以使用高度作为强度
                }
            }
        }
        
        // 发布激光扫描消息
        laser_scan_pub_->publish(std::move(scan_msg));
    }
    
    // ROS2 接口
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    
    // 参数
    std::string output_frame_;
    double min_height_;
    double max_height_;
    double angle_min_;
    double angle_max_;
    double angle_increment_;
    double scan_time_;
    double range_min_;
    double range_max_;
    size_t num_readings_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Lidar2DConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}