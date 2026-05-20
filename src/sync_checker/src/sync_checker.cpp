#include <memory>
#include <deque>
#include <string>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"

class SyncChecker : public rclcpp::Node
{
public:
    SyncChecker()
    : Node("sync_checker")
    , tf_buffer_(this->get_clock())
    , tf_listener_(tf_buffer_)
    {
        // 声明参数
        this->declare_parameter<std::string>("path_frame_id", "odom");
        this->declare_parameter<int>("max_path_length", 2000);
        this->declare_parameter<double>("time_tolerance", 0.08);
        
        path_frame_id_ = this->get_parameter("path_frame_id").as_string();
        max_path_length_ = this->get_parameter("max_path_length").as_int();
        time_tolerance_ = this->get_parameter("time_tolerance").as_double();
        
        // 创建发布者和订阅者
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("odom_path", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::QoS(10).reliable(),
            std::bind(&SyncChecker::odomCallback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            std::bind(&SyncChecker::scanCallback, this, std::placeholders::_1));
        
        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&SyncChecker::statsTimerCallback, this));
        
        // 初始化路径消息
        path_msg_.header.frame_id = path_frame_id_;
        
        RCLCPP_INFO(this->get_logger(), "SyncChecker node started.");
        RCLCPP_INFO(this->get_logger(), "Path frame: %s, max path length: %d, time tolerance: %.3f s",
                    path_frame_id_.c_str(), max_path_length_, time_tolerance_);
    }
    
private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string path_frame_id_;

    nav_msgs::msg::Path path_msg_;
    std::deque<geometry_msgs::msg::PoseStamped> pose_queue_;  // 用于限制路径长度
    std::deque<rclcpp::Time> odom_time_queue_;                 // 用于与 scan 匹配
    bool has_odom_ = false;                                    // 是否收到过里程计
    int max_path_length_ = 2000;
    double time_tolerance_ = 0.08;

    size_t scan_count_ = 0;
    size_t sync_ok_count_ = 0;
    size_t sync_warn_count_ = 0;
    double sum_time_diff_ = 0.0;
    double min_time_diff_ = std::numeric_limits<double>::max();
    double max_time_diff_ = 0.0;
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 记录里程计时间并标记已收到里程计
        odom_time_queue_.push_back(msg->header.stamp);
        if (odom_time_queue_.size() > 2000) {
            odom_time_queue_.pop_front();
        }
        has_odom_ = true;

        // 构造 PoseStamped
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;                // 保留原始 stamp 和 frame_id
        pose.pose = msg->pose.pose;

        if (path_frame_id_ != msg->header.frame_id) {
            try {
                geometry_msgs::msg::PoseStamped transformed_pose;
                tf_buffer_.transform(pose, transformed_pose, path_frame_id_, tf2::durationFromSec(0.1));
                pose = transformed_pose;
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(),
                            "Failed to transform odom pose from '%s' to '%s': %s. Using raw odom pose.",
                            msg->header.frame_id.c_str(), path_frame_id_.c_str(), ex.what());
            }
        }

        // 加入路径队列
        pose_queue_.push_back(pose);
        if (pose_queue_.size() > static_cast<size_t>(max_path_length_)) {
            pose_queue_.pop_front();
        }

        // 更新 Path 消息
        path_msg_.header.stamp = this->now();
        path_msg_.poses.assign(pose_queue_.begin(), pose_queue_.end());

        // 发布路径
        path_pub_->publish(path_msg_);
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!has_odom_ || odom_time_queue_.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5.0,
                                 "Received scan but no odometry yet. Ignoring sync check.");
            return;
        }

        rclcpp::Time scan_time = msg->header.stamp;
        double best_diff = std::numeric_limits<double>::max();
        rclcpp::Time best_odom_time(0);

        // 找到与 scan 时间戳最接近的 odom 时间
        for (const auto &odom_time : odom_time_queue_) {
            double diff = std::fabs((scan_time - odom_time).seconds());
            if (diff < best_diff) {
                best_diff = diff;
                best_odom_time = odom_time;
            }
        }

        scan_count_++;
        sum_time_diff_ += best_diff;
        min_time_diff_ = std::min(min_time_diff_, best_diff);
        max_time_diff_ = std::max(max_time_diff_, best_diff);

        if (best_diff > time_tolerance_) {
            sync_warn_count_++;
            RCLCPP_WARN(this->get_logger(),
                        "Time sync issue: scan stamp = %.3f, closest odom stamp = %.3f, diff = %.3f s (threshold = %.3f)",
                        scan_time.seconds(), best_odom_time.seconds(), (scan_time - best_odom_time).seconds(), time_tolerance_);
        } else {
            sync_ok_count_++;
            RCLCPP_DEBUG(this->get_logger(),
                         "Time sync OK: scan stamp = %.3f, closest odom stamp = %.3f, diff = %.3f s",
                         scan_time.seconds(), best_odom_time.seconds(), (scan_time - best_odom_time).seconds());
        }

        // 清理过期的 odom 数据，保留最近 5 秒的时间戳
        while (!odom_time_queue_.empty() && (scan_time - odom_time_queue_.front()).seconds() > 5.0) {
            odom_time_queue_.pop_front();
        }
    }

    void statsTimerCallback()
    {
        if (scan_count_ == 0) {
            RCLCPP_INFO(this->get_logger(), "No scan sync samples yet.");
            return;
        }

        double avg_time_diff = sum_time_diff_ / static_cast<double>(scan_count_);
        RCLCPP_INFO(this->get_logger(),
                    "Scan/Odom sync stats: samples=%zu, ok=%zu, warn=%zu, avg=%.4f s, min=%.4f s, max=%.4f s",
                    scan_count_, sync_ok_count_, sync_warn_count_, avg_time_diff, min_time_diff_, max_time_diff_);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SyncChecker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}