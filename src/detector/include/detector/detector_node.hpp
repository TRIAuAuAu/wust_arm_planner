#ifndef EXCHANGE_SLOT__DETECTOR_NODE_HPP_
#define EXCHANGE_SLOT__DETECTOR_NODE_HPP_

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "detector/detector.hpp"

namespace exchange_slot
{
class ExchangeSlotDetectorNode : public rclcpp::Node
{
public:
  explicit ExchangeSlotDetectorNode(const rclcpp::NodeOptions & options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  void createDebugPublishers();
  void destroyDebugPublishers();
  void publishMarkers(const geometry_msgs::msg::Pose & pose, const std_msgs::msg::Header & header);

  // 核心算法类
  std::unique_ptr<ExchangeSlotDetector> detector_;

  // 发布者
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // TF 广播器 (可选，用于在 RViz 中观察目标坐标系)
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 订阅者
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  // 调试与动态参数
  bool debug_;
  bool no_hardware_;
  image_transport::Publisher result_img_pub_;
  image_transport::Publisher binary_img_pub_;
  
  // Marker 模板
  visualization_msgs::msg::Marker base_marker_;
  visualization_msgs::msg::Marker cylinder_marker_;
};
} // namespace exchange_slot

#endif