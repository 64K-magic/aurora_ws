#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace slamware_nav2_bridge
{

class MapOdomBridgeNode : public rclcpp::Node
{
public:
  MapOdomBridgeNode()
  : Node("map_odom_bridge_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    robot_pose_topic_ = declare_parameter<std::string>(
      "robot_pose_topic", "/slamware_ros_sdk_server_node/robot_pose");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/slamware_ros_sdk_server_node/odom");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.5);
    const double publish_rate = declare_parameter<double>("publish_rate_hz", 50.0);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    robot_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      robot_pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MapOdomBridgeNode::robotPoseCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MapOdomBridgeNode::odomCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(publish_rate, 1.0));
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MapOdomBridgeNode::publishTimerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "map_odom bridge: robot_pose=%s odom=%s -> TF %s->%s and %s->%s",
      robot_pose_topic_.c_str(), odom_topic_.c_str(),
      map_frame_.c_str(), odom_frame_.c_str(), odom_frame_.c_str(), robot_frame_.c_str());
  }

private:
  void robotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_robot_pose_ = msg;
    tryPublishLocked();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = msg;
    tryPublishLocked();
  }

  void publishTimerCallback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_valid_tf_) {
      const auto stamp = now();
      last_map_odom_.header.stamp = stamp;
      last_odom_base_.header.stamp = stamp;
      tf_broadcaster_->sendTransform(last_map_odom_);
      tf_broadcaster_->sendTransform(last_odom_base_);
    }
  }

  bool lookupPoseInMapFrame(
    const geometry_msgs::msg::PoseStamped & pose_in,
    geometry_msgs::msg::PoseStamped & pose_map)
  {
    if (pose_in.header.frame_id == map_frame_) {
      pose_map = pose_in;
      pose_map.header.frame_id = map_frame_;
      return true;
    }
    try {
      pose_map = tf_buffer_.transform(
        pose_in, map_frame_, tf2::durationFromSec(transform_tolerance_));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "robot_pose %s -> %s: %s",
        pose_in.header.frame_id.c_str(), map_frame_.c_str(), ex.what());
      return false;
    }
  }

  void tryPublishLocked()
  {
    if (!latest_robot_pose_ || !latest_odom_) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose_map;
    if (!lookupPoseInMapFrame(*latest_robot_pose_, pose_map)) {
      return;
    }

    const auto & odom = *latest_odom_;
    if (odom.child_frame_id != robot_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "odom child_frame_id is '%s', expected '%s'",
        odom.child_frame_id.c_str(), robot_frame_.c_str());
    }

    tf2::Transform tf_map_base;
    tf2::fromMsg(pose_map.pose, tf_map_base);

    tf2::Transform tf_odom_base;
    tf2::fromMsg(odom.pose.pose, tf_odom_base);

    const tf2::Transform tf_map_odom = tf_map_base * tf_odom_base.inverse();

    geometry_msgs::msg::TransformStamped map_odom;
    map_odom.header.stamp = odom.header.stamp;
    map_odom.header.frame_id = map_frame_;
    map_odom.child_frame_id = odom_frame_;
    map_odom.transform = tf2::toMsg(tf_map_odom);

    geometry_msgs::msg::TransformStamped odom_base;
    odom_base.header.stamp = odom.header.stamp;
    odom_base.header.frame_id = odom_frame_;
    odom_base.child_frame_id = odom.child_frame_id.empty() ? robot_frame_ : odom.child_frame_id;
    odom_base.transform = tf2::toMsg(tf_odom_base);

    last_map_odom_ = map_odom;
    last_odom_base_ = odom_base;
    has_valid_tf_ = true;
    tf_broadcaster_->sendTransform(map_odom);
    tf_broadcaster_->sendTransform(odom_base);
  }

  std::string robot_pose_topic_;
  std::string odom_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string robot_frame_;
  double transform_tolerance_{0.5};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped::SharedPtr latest_robot_pose_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  geometry_msgs::msg::TransformStamped last_map_odom_;
  geometry_msgs::msg::TransformStamped last_odom_base_;
  bool has_valid_tf_{false};
};

}  // namespace slamware_nav2_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slamware_nav2_bridge::MapOdomBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
