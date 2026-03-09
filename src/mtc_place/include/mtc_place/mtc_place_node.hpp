#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_msgs//srv/get_planning_scene.hpp>
#include "detector_interfaces/srv/get_detector_state.hpp"
#include "detector_interfaces/srv/set_detector_state.hpp"

using GetDetectorState = detector_interfaces::srv::GetDetectorState;
using SetDetectorState = detector_interfaces::srv::SetDetectorState;

namespace mtc_place
{
enum class State { LOST, TRACKING, LOCKED, PLANNING, EXECUTING };
class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  /** TaskExecutor interface **/
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  /** Trigger task manually **/
  void doTask();
  
  /** Planning scene setup **/
  void setupPlanningScene(const geometry_msgs::msg::PoseStamped &slot_pose);

private:
  /** Callback **/
  void slotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  /** Task constructor **/
  moveit::task_constructor::Task createTask();
  moveit::task_constructor::Task createTestTask();
  /** Detector State **/
void setDetectorState(uint8_t state);

private:
  rclcpp::Node::SharedPtr node_;

  std::string arm_group_;
  std::string hand_frame_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr slot_pose_sub_;

  // 状态机管理
  rclcpp::Client<GetDetectorState>::SharedPtr get_state_client_;
  rclcpp::Client<SetDetectorState>::SharedPtr set_state_client_;
  rclcpp::TimerBase::SharedPtr task_timer_;
  void handleGetStateResponse(rclcpp::Client<GetDetectorState>::SharedFuture future);
  void startPlanningPipeline();
  // 互斥回调组
  rclcpp::CallbackGroup::SharedPtr cb_group_service_;
  rclcpp::CallbackGroup::SharedPtr cb_group_timer_;

  geometry_msgs::msg::PoseStamped latest_slot_pose_;
  bool task_running_ = false;

  moveit::task_constructor::Task task_;
  
};
geometry_msgs::msg::Pose tfToPose(const tf2::Transform &tf);
}  // namespace mtc_place
