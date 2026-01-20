#ifndef EXCHANGE_SLOT__DETECTOR_NODE_HPP_
#define EXCHANGE_SLOT__DETECTOR_NODE_HPP_

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "detector/detector.hpp"

namespace exchange_slot
{
enum class State { LOST, TRACKING, LOCKED, PLANNING };

class ExchangeSlotDetectorNode : public rclcpp::Node
{
public:
  explicit ExchangeSlotDetectorNode(const rclcpp::NodeOptions & options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  void createDebugPublishers();
  void destroyDebugPublishers();
  void publishMarkers(const geometry_msgs::msg::Pose & pose, const std_msgs::msg::Header & header);
  
  // 稳定性判定：增加角度约束
  bool isStable(const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2);

  // 识别器
  std::unique_ptr<ExchangeSlotDetector> detector_;

  // 发布者与订阅者
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  // 服务通信
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr lock_service_;
  void handleLockService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  // 参数
  bool debug_;
  bool no_hardware_;
  int stable_threshold_;    // 进入 LOCKED 的稳定帧数
  double pos_tolerance_;    // 位置公差 (m)
  double ori_tolerance_;    // 角度公差 (度)
  int lost_buffer_limit_;   // 允许丢失的最大连续帧数
  bool force_lock_;

  image_transport::Publisher result_img_pub_;
  image_transport::Publisher binary_img_pub_;
  
  visualization_msgs::msg::Marker base_marker_;
  visualization_msgs::msg::Marker cylinder_marker_;

  // 状态机核心变量
  State current_state_ = State::LOST;
  geometry_msgs::msg::Pose locked_pose_;
  int stable_frames_ = 0;   // 稳定计数
  int lost_frames_counter_ = 0; // 丢失缓冲计数
};
} 

#endif