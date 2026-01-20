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
  getParams();
  
  joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
  fake_joint_positions_.assign(7, 0.0);
  sim_target_positions_.assign(7, 0.0);
  last_sim_update_time_ = this->now(); // 初始化时间戳

    if (use_fake_hardware_) {
      RCLCPP_WARN(get_logger(), "!!! RUNNING IN FAKE MODE !!!");
      // 根据参数计算频率
      int interval_ms = static_cast<int>(1000.0 / state_publish_rate_);
      joint_state_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms),
        std::bind(&WustArmDriver::publishFakeJointStates, this));
    } else{
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
  auto now = this->now();
  double dt = (now - last_sim_update_time_).seconds();
  last_sim_update_time_ = now;

  // 安全检查：防止 dt 异常（比如初次运行或系统时间跳变）
  if (dt <= 0.0 || dt > 0.5) {
    dt = 0.02; // 强制设为一个合理的步长 (50Hz)
  }

  // 1. 线性平滑插值 (一阶低通滤波模拟)
  // 这里的 0.1 可以改为从参数读取的 sim_smooth_factor
  // 这种方式不会因为 error 过大产生震荡，运动更丝滑
  for (size_t i = 0; i < 7; ++i) {
    double diff = sim_target_positions_[i] - fake_joint_positions_[i];
    if (std::abs(diff) < 0.0001) {
      fake_joint_positions_[i] = sim_target_positions_[i];
    } else {
      // 这里的 10.0 是增益系数，值越大，仿真追随速度越快
      fake_joint_positions_[i] += diff * (10.0 * dt); 
    }
  }

  // 2. 发布 JointState
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now;
  msg.name = joint_names_;
  msg.position = fake_joint_positions_;
  joint_state_pub_->publish(msg);
}

void WustArmDriver::getParams()
{
  // 声明参数
  this->declare_parameter<bool>("use_fake_hardware", true);
  this->declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 115200);
  this->declare_parameter<double>("sim_max_speed", 1.5);
  this->declare_parameter<double>("state_publish_rate", 50.0);
  this->declare_parameter<double>("goal_tolerance", 0.01);
  this->declare_parameter<double>("goal_timeout", 8.0);
  this->declare_parameter<int>("controller_freq", 50);
  this->declare_parameter<bool>("debug", true);

  // 获取参数值并赋值给成员变量
  use_fake_hardware_    = this->get_parameter("use_fake_hardware").as_bool();
  device_name_          = this->get_parameter("device_name").as_string();
  int baud_rate         = this->get_parameter("baud_rate").as_int();
  sim_max_speed_        = this->get_parameter("sim_max_speed").as_double();
  state_publish_rate_   = this->get_parameter("state_publish_rate").as_double();
  goal_tolerance_       = this->get_parameter("goal_tolerance").as_double();
  goal_timeout_         = this->get_parameter("goal_timeout").as_double();
  controller_freq_      = this->get_parameter("controller_freq").as_int();
  debug_                =     this->get_parameter("debug").as_bool();

  // 打印确认
  // RCLCPP_INFO(this->get_logger(), "Mode: %s, Freq: %d, Tolerance: %.3f", 
  //             use_fake_hardware_ ? "FAKE" : "REAL", controller_freq_, goal_tolerance_);

  // 串口配置
  auto fc = drivers::serial_driver::FlowControl::NONE;
  auto pt = drivers::serial_driver::Parity::NONE;
  auto sb = drivers::serial_driver::StopBits::ONE;
  device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(
    baud_rate, fc, pt, sb);
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

  int sleep_ms = 1000 / controller_freq_;

  for (size_t i = 0; i < goal->trajectory.points.size(); ++i) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    }

    const auto & point = goal->trajectory.points[i];
    
    if (use_fake_hardware_) {
      // 仿真模式：只更新“下位机目标”，不改 fake_joint_positions_
      sim_target_positions_ = point.positions;
    } else {
      // 真实模式：发送串口包
      SendPacket packet;
      for (size_t j = 0; j < 7; ++j) {
        packet.target_joint_positions[j] = static_cast<float>(point.positions[j]);
      }
      crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
      serial_driver_->port()->send(toVector(packet));
    }

    // 反馈当前真实的物理位置（来自传感器或仿真插值）
    feedback->actual.positions = fake_joint_positions_;
    feedback->header.stamp = this->now();
    goal_handle->publish_feedback(feedback);

    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
  }

  // 判断是否到达目标点
  RCLCPP_INFO(get_logger(), "Trajectory sent, waiting for joints to settle...");
  auto start_time = this->now();
  const auto & final_target = goal->trajectory.points.back().positions;

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) { goal_handle->canceled(result); return; }

    if ((this->now() - start_time).seconds() > goal_timeout_) {
      RCLCPP_ERROR(get_logger(), "Goal Failed: Joints did not reach tolerance in time.");
      result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
      goal_handle->abort(result);
      return;
    }

    // 这里 check_goal_reached 会检查物理反馈 fake_joint_positions_
    if (check_goal_reached(final_target)) {
      break; 
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200)); // 额外静止 200ms，防止因为抖动判定失败了
  result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
  goal_handle->succeed(result);
  RCLCPP_INFO(get_logger(), "Goal Reached Succeeded!");
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