#include "ros2_lidar_2d_converter/laserscan_to_pointcloud_node.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace pointcloud_to_laserscan
{

LaserScanToPointCloudNode::LaserScanToPointCloudNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("laserscan_to_pointcloud", options)
{
  target_frame_ = this->declare_parameter("target_frame", "");
  tolerance_ = this->declare_parameter("transform_tolerance", 0.01);
  // TODO(hidmic): adjust default input queue size based on actual concurrency levels
  // achievable by the associated executor
  input_queue_size_ = this->declare_parameter(
    "queue_size", static_cast<int>(std::thread::hardware_concurrency()));

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::SensorDataQoS());

  using std::placeholders::_1;
  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty()) {
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    message_filter_ = std::make_unique<MessageFilter>(
      sub_, *tf2_, target_frame_, input_queue_size_,
      this->get_node_logging_interface(),
      this->get_node_clock_interface());
    message_filter_->registerCallback(
      std::bind(
        &LaserScanToPointCloudNode::scanCallback, this, _1));
  } else {  // otherwise setup direct subscription
    sub_.registerCallback(std::bind(&LaserScanToPointCloudNode::scanCallback, this, _1));
  }

  subscription_listener_thread_ = std::thread(
    std::bind(&LaserScanToPointCloudNode::subscriptionListenerThreadLoop, this));
}

LaserScanToPointCloudNode::~LaserScanToPointCloudNode()
{
  alive_.store(true);
  subscription_listener_thread_.join();
}

void LaserScanToPointCloudNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();

  const std::chrono::milliseconds timeout(100);
  while (rclcpp::ok(context) && alive_.load()) {
    int subscription_count = pub_->get_subscription_count() +
      pub_->get_intra_process_subscription_count();
    if (subscription_count > 0) {
      if (!sub_.getSubscriber()) {
        RCLCPP_INFO(
          this->get_logger(),
          "Got a subscriber to pointcloud, starting laserscan subscriber");
        rclcpp::SensorDataQoS qos;
        qos.keep_last(input_queue_size_);
        sub_.subscribe(this, "scan_in", qos.get_rmw_qos_profile());
      }
    } else if (sub_.getSubscriber()) {
      RCLCPP_INFO(
        this->get_logger(),
        "No subscribers to pointcloud, shutting down laserscan subscriber");
      sub_.unsubscribe();
    }
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }
  sub_.unsubscribe();
}

void LaserScanToPointCloudNode::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
  auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();

  projector_.projectLaser(*scan_msg, *cloud_msg);

  // Transform cloud if necessary
  if (!target_frame_.empty() && cloud_msg->header.frame_id != target_frame_) {
    try {
      *cloud_msg = tf2_->transform(*cloud_msg, target_frame_, tf2::durationFromSec(tolerance_));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
      return;
    }
  }
  pub_->publish(std::move(cloud_msg));
}

}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::LaserScanToPointCloudNode)
