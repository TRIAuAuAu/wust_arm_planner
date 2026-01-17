#include "wust_arm_driver/wust_arm_driver.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace wust_arm_driver
{
WustArmDriver::WustArmDriver(const rclcpp::NodeOptions & options)
: Node("wust_arm_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Starting WustArmDriver!");

  // 1. 初始化参数和串口配置
  getParams();
  joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
  fake_joint_positions_.resize(7, 0.0);
  // 2. 尝试打开串口
  if (use_fake_hardware_) {
    RCLCPP_WARN(get_logger(), "Using FAKE hardware mode (no serial port)");

    joint_state_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&WustArmDriver::publishFakeJointStates, this));
  } else {
    try {
      serial_driver_->init_port(device_name_, *device_config_);
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&WustArmDriver::receiveData, this);
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(get_logger(), "Serial init failed: %s", ex.what());
      throw;
    }
  }

  // 3. 创建 ROS 接口
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    this, "main_controller/follow_joint_trajectory",
    std::bind(&WustArmDriver::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&WustArmDriver::handle_cancel, this, std::placeholders::_1),
    std::bind(&WustArmDriver::handle_accepted, this, std::placeholders::_1));
}

WustArmDriver::~WustArmDriver()
{
  if (!use_fake_hardware_) {
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
    if (serial_driver_ && serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}


void WustArmDriver::publishFakeJointStates()
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();
  msg.name = joint_names_;
  msg.position = fake_joint_positions_;
  joint_state_pub_->publish(msg);
}

void WustArmDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  device_name_ = declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
  baud_rate = declare_parameter<int>("baud_rate", 115200);
  
  // 这里省略第五个参数数据位，自动采用默认值 (8位)
  device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(
    baud_rate, fc, pt, sb);

  use_fake_hardware_ = declare_parameter<bool>("use_fake_hardware", true);
  goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.01);
  goal_timeout_ = declare_parameter<double>("goal_timeout", 5.0);
  controller_freq_ = declare_parameter<int>("controller_freq", 50); // 默认 50Hz
  debug_ = declare_parameter<bool>("debug", false);
}

// Action Server 回调实现
rclcpp_action::GoalResponse WustArmDriver::handle_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJointTrajectory::Goal>)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WustArmDriver::handle_cancel(
  const std::shared_ptr<GoalHandleFollowJointTrajectory>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WustArmDriver::handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  // 在独立线程中执行，避免阻塞主循环
  std::thread{std::bind(&WustArmDriver::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void WustArmDriver::execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<FollowJointTrajectory::Result>();
  auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();

  // 计算发送间隔
  int sleep_ms = 1000 / controller_freq_;
  
  if (debug_) {
    RCLCPP_INFO(get_logger(), "Executing trajectory with %zu points at %d Hz", 
                goal->trajectory.points.size(), controller_freq_);
  }

  // 发送轨迹点
  for (size_t i = 0; i < goal->trajectory.points.size(); ++i) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    }

    const auto & point = goal->trajectory.points[i];
    
    // 更新位置（仿真或发送串口）
    if (use_fake_hardware_) {
      fake_joint_positions_ = point.positions;
    } else {
      SendPacket packet;
      for (size_t j = 0; j < 7; ++j) {
        packet.target_joint_positions[j] = static_cast<float>(point.positions[j]);
      }
      crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
      serial_driver_->port()->send(toVector(packet));
    }

    // 只有在 debug 模式下才频繁打印反馈，节省正式运行时的 CPU 消耗
    if (debug_ && i % 10 == 0) {
      RCLCPP_INFO(get_logger(), "Sending point %zu/%zu", i, goal->trajectory.points.size());
    }

    // 发布中间反馈给 MoveIt
    feedback->actual.positions = fake_joint_positions_;
    feedback->header.stamp = this->now();
    goal_handle->publish_feedback(feedback);

    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
  }

  // 闭环到达判断
  auto start_time = this->now();
  const auto & final_target = goal->trajectory.points.back().positions;
  bool reached = false;

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) { goal_handle->canceled(result); return; }

    // 超时检查
    if ((this->now() - start_time).seconds() > goal_timeout_) {
      RCLCPP_ERROR(get_logger(), "Trajectory Timeout! Failed to reach tolerance.");
      result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
      goal_handle->abort(result);
      return;
    }

    if (check_goal_reached(final_target)) {
      reached = true;
      break;
    }
    // 检查频率不必太高
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (reached) {
    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    goal_handle->succeed(result);
    if (debug_) RCLCPP_INFO(get_logger(), "Goal Succeeded!");
  }
}

// 辅助函数：判断所有关节是否进入容差范围
bool WustArmDriver::check_goal_reached(const std::vector<double> & target_positions)
{
  for (size_t i = 0; i < 7; ++i) {
    if (std::abs(fake_joint_positions_[i] - target_positions[i]) > goal_tolerance_) {
      return false;
    }
  }
  return true;
}

void WustArmDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);

      if (header[0] == 0x5A) { 
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);
        data.insert(data.begin(), header[0]);

        ReceivePacket packet = fromVector(data);

        if (crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet))) {
          auto msg = sensor_msgs::msg::JointState();
          // 使用 this->now() 保证 TF 同步
          msg.header.stamp = this->now();
          msg.name = joint_names_;
          for (int i = 0; i < 7; ++i) {
            msg.position.push_back(packet.current_joint_positions[i]);
          }
          joint_state_pub_->publish(msg);
        }
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 20, "Receive error: %s", ex.what());
      // 避免死循环占用 CPU
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

}  // namespace wust_arm_driver

RCLCPP_COMPONENTS_REGISTER_NODE(wust_arm_driver::WustArmDriver)