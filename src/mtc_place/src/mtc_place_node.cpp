#include "mtc_place/mtc_place_node.hpp"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mtc_place
{

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
{ 
  /* ---------- node ---------- */
  node_ = std::make_shared<rclcpp::Node>("mtc_place_node", options);

  /* ---------- parameters ---------- */
  node_->declare_parameter("arm_group", "main");
  node_->declare_parameter("hand_frame", "LINK_TCP");

  arm_group_  = node_->get_parameter("arm_group").as_string();
  hand_frame_ = node_->get_parameter("hand_frame").as_string();

  /* ---------- callback_group ---------- */
  cb_group_service_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_timer_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  /* ---------- interfaces ---------- */
  slot_pose_sub_ =
    node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/exchange_slot/slot_pose", 10,
      std::bind(&MTCTaskNode::slotPoseCallback, this, std::placeholders::_1));
  get_state_client_ = node_->create_client<GetDetectorState>(
    "/detector/get_state", rmw_qos_profile_services_default, cb_group_service_);
  
  set_state_client_ = node_->create_client<SetDetectorState>(
    "/detector/set_state", rmw_qos_profile_services_default, cb_group_service_);
  RCLCPP_INFO(node_->get_logger(), "MTCTaskNode constructed");
  task_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(500),
  std::bind(&MTCTaskNode::doTask, this),
  cb_group_timer_);
}
/* ===================== planning scene ===================== */

void MTCTaskNode::setupPlanningScene(const geometry_msgs::msg::PoseStamped &slot_pose_stamped)
{
    moveit::planning_interface::PlanningSceneInterface psi;

    double square_size = 0.10;
    double base_height = 0.01;
    double cylinder_height = 0.10;
    double cylinder_radius = 0.018;

    // ---------- 确认 slot_pose_stamped 已经在 base_link 下 ----------
    geometry_msgs::msg::PoseStamped slot_pose = slot_pose_stamped;
    slot_pose.header.frame_id = "base_link"; // 强制 frame_id 为 base_link

    // ---------- 1. 底座碰撞体 ----------
    moveit_msgs::msg::CollisionObject base;
    base.id = "slot_base";
    base.header.frame_id = "base_link";
    base.primitives.resize(1);
    base.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    base.primitives[0].dimensions = {square_size, square_size, base_height};
    base.pose = slot_pose.pose;
    base.operation = base.ADD;
    psi.applyCollisionObject(base);

    // ---------- 2. 圆柱碰撞体 ----------
    moveit_msgs::msg::CollisionObject cylinder;
    cylinder.id = "slot_cylinder";
    cylinder.header.frame_id = "base_link";
    cylinder.primitives.resize(1);
    cylinder.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder.primitives[0].dimensions = {cylinder_height, cylinder_radius};

    // 使用 tf2::Transform 计算圆柱相对于底座的偏移，避免坐标系偏移导致的误差
    tf2::Transform tf_base, tf_offset, tf_cylinder;
    tf2::fromMsg(slot_pose.pose, tf_base); // 底座位姿
    tf_offset.setIdentity();
    tf_offset.setOrigin(tf2::Vector3(0, 0, base_height / 2.0 + cylinder_height / 2.0)); // z 偏移
    tf_cylinder = tf_base * tf_offset; // 底座+偏移

    cylinder.pose = tfToPose(tf_cylinder);
    cylinder.operation = cylinder.ADD;
    psi.applyCollisionObject(cylinder);

    // ---------- 3. 附着 hollow_cylinder ----------
    moveit_msgs::msg::CollisionObject hollow;
    hollow.id = "hollow_cylinder";
    // 注意：根据之前的日志，请确保 hand_frame_ 对应的是 "LINK_TCP" 而不是关节名
    hollow.header.frame_id = hand_frame_; 
    
    hollow.primitives.resize(1);
    hollow.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    
    double h = 0.15;
    double r = 0.0325; // 更新半径为 0.0325
    hollow.primitives[0].dimensions = {h, r};

    tf2::Transform tf_hollow;
    
    /* 计算位移：
       1. X 方向：移动 0.0366，使圆柱中心对齐 LINK7 的 Z 轴。
       2. Y 方向：0。
       3. Z 方向：从 hand_frame_ 往回退，直到靠近 LINK7。
          距离为 0.081603。若要相切，中心点应在 0.081603 - r 处。
          但为了保险（防止碰撞检测过敏），我们先移到 -0.05 附近。
    */
    double offset_x = 0.0366;
    double offset_z = -(0.081603 - r); // 往回移动，使侧面与 LINK7 平面相切

    tf_hollow.setOrigin(tf2::Vector3(offset_x, 0, offset_z)); 

    // 旋转：让圆柱长轴沿 Y 轴
    tf2::Quaternion q;
    q.setRPY(M_PI / 2.0, 0, 0); 
    tf_hollow.setRotation(q);

    hollow.pose = tfToPose(tf_hollow); 
    hollow.operation = hollow.ADD;

    // 附着
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.object = hollow;
    attached.link_name = hand_frame_;
    attached.touch_links = {hand_frame_,"LINK7"}; // 与手爪和LINK7允许接触
    psi.applyAttachedCollisionObject(attached);
    // 等待附着生效
    std::map<std::string, moveit_msgs::msg::AttachedCollisionObject> objects;
    int attempts = 0;
    while (objects.find("hollow_cylinder") == objects.end() && attempts < 5) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        objects = psi.getAttachedObjects({"hollow_cylinder"});
        attempts++;
    }
    RCLCPP_DEBUG(node_->get_logger(), "Object 'hollow_cylinder' is now attached and visible.");
}

/* ===================== executor interface ===================== */

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

/* ===================== callbacks ===================== */

void MTCTaskNode::slotPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  latest_slot_pose_ = *msg;

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,"Received slot pose");
}

/* ===================== main task entry ===================== */
void MTCTaskNode::handleGetStateResponse(
  rclcpp::Client<GetDetectorState>::SharedFuture future)
{
  RCLCPP_DEBUG(node_->get_logger(), "Received state response from detector!");
  uint8_t state = future.get()->state;

  if (state != static_cast<uint8_t>(State::LOCKED))
    return;

  startPlanningPipeline();
  RCLCPP_DEBUG(node_->get_logger(), "Got detector state response");
}
void MTCTaskNode::startPlanningPipeline()
{
  if (task_running_) return;
  task_running_ = true;
  // 检查 MoveGroup 服务是否在线 ---
  auto get_scene_client = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  if (!get_scene_client->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_DEBUG(node_->get_logger(), "MoveGroup not ready, resetting flag...");
    task_running_ = false; // 复位
    return; 
  }
  RCLCPP_INFO(node_->get_logger(), "Start PLANNING");

  setDetectorState(static_cast<uint8_t>(State::PLANNING));

  setupPlanningScene(latest_slot_pose_);
  task_ = createTestTask();
  task_.enableIntrospection();

  try {
    task_.init();
    RCLCPP_DEBUG(node_->get_logger(), "MTC Task init successfully");
  } catch (const moveit::task_constructor::InitStageException& e) {
    RCLCPP_ERROR(node_->get_logger(), "MTC Task init failed: %s", e.what());
    std::cout << task_ << std::endl; // debug
    task_running_ = false;
    return;
  }

  if (!task_.plan(5)) {
    task_running_ = false;
    setDetectorState(static_cast<uint8_t>(State::LOCKED));
    RCLCPP_ERROR(node_->get_logger(), "Task planning failed!");
    std::cout << task_ << std::endl; // debug
    return;
  }

  auto solution = task_.solutions().front();
  task_.introspection().publishSolution(*solution);
  // std::cout << task_ << std::endl; // debug
  
  setDetectorState(static_cast<uint8_t>(State::EXECUTING));

  auto result = task_.execute(*solution);

  if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    setDetectorState(static_cast<uint8_t>(State::LOST));
  else
    setDetectorState(static_cast<uint8_t>(State::LOCKED));

  task_running_ = false;
}
void MTCTaskNode::doTask()
{
  if (task_running_) return;

  // 检查服务是否在线
  if (!get_state_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Detector Service not ready...");
    return;
  }

  auto req = std::make_shared<GetDetectorState::Request>();
  get_state_client_->async_send_request(
    req,
    std::bind(&MTCTaskNode::handleGetStateResponse, this, std::placeholders::_1)
  );
}



/* ===================== MTC ===================== */

moveit::task_constructor::Task MTCTaskNode::createTask()
{
  moveit::task_constructor::Task task;
  task.stages()->setName("place_hollow_cylinder");

  task.loadRobotModel(node_);
  task.setProperty("group", arm_group_);
  task.setProperty("ik_frame", hand_frame_);

  auto pipeline = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node_);
  auto cartesian = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
  cartesian->setStepSize(0.003);
  cartesian->setMaxVelocityScalingFactor(0.2);
  cartesian->setMaxAccelerationScalingFactor(0.2);

  moveit::task_constructor::Stage* current_state = nullptr;
  {
    auto cs = std::make_unique<moveit::task_constructor::stages::CurrentState>("current");
    current_state = cs.get();
    task.add(std::move(cs));
  }

  // ---------- Connect ----------
  {
    auto connect = std::make_unique<moveit::task_constructor::stages::Connect>(
        "connect", moveit::task_constructor::stages::Connect::GroupPlannerVector{
          {arm_group_, pipeline}});
    connect->setTimeout(5.0);
    task.add(std::move(connect));
  }

  // ---------- Place ----------
  {
    auto place = std::make_unique<moveit::task_constructor::SerialContainer>("place");
    task.properties().exposeTo(place->properties(), {"group", "ik_frame"});

    // --- 1. 预放置姿态 ---
    {
      auto gen = std::make_unique<moveit::task_constructor::stages::GeneratePose>("generate_pre_place");

      geometry_msgs::msg::PoseStamped pre_place = latest_slot_pose_;
      pre_place.pose.position.z += 0.05 + 0.10/2.0 + 0.15/2.0; // 圆柱 + hollow偏移
      gen->setPose(pre_place);
      gen->setMonitoredStage(current_state);

      auto ik = std::make_unique<moveit::task_constructor::stages::ComputeIK>("pre_place_ik", std::move(gen));
      ik->setIKFrame(hand_frame_);
      ik->setMaxIKSolutions(16);

      place->insert(std::move(ik));
    }

    // --- 2. 插入动作（沿 slot z 负方向） ---
    {
      auto insert = std::make_unique<moveit::task_constructor::stages::MoveRelative>("insert", cartesian);
      insert->setIKFrame(hand_frame_);
      insert->setMinMaxDistance(0.04, 0.06);

      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = latest_slot_pose_.header.frame_id;
      dir.vector.z = -1.0;
      insert->setDirection(dir);

      // 碰撞控制
      // 允许到达预放置姿态时 slot_base + slot_cylinder 碰撞
      insert->properties().set("collision_objects", std::vector<std::string>{"slot_base"});
      insert->properties().set("allowed_collision_objects", std::vector<std::string>{"slot_cylinder"});

      place->insert(std::move(insert));
    }

    // --- 3. Detach hollow_cylinder ---
    {
      auto detach = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("detach_hollow");
      detach->detachObject("hollow_cylinder", hand_frame_);
      place->insert(std::move(detach));
    }

    task.add(std::move(place));
  }

  return task;
}



/* ===================== detector lock ===================== */
moveit::task_constructor::Task MTCTaskNode::createTestTask()
{
  using namespace moveit::task_constructor;
  Task task;
  task.stages()->setName("test_state_machine_task");
  task.loadRobotModel(node_);
  task.setProperty("group", arm_group_);
  task.setProperty("ik_frame", hand_frame_); // 明确 IK 框架

  auto pipeline = std::make_shared<solvers::PipelinePlanner>(node_);

  // Current State
  task.add(std::make_unique<stages::CurrentState>("current"));

  // Move to 90deg
  {
    auto move = std::make_unique<stages::MoveTo>("joint1_90deg", pipeline);
    move->setGroup(arm_group_);
    std::map<std::string, double> target;
    target["joint1"] = M_PI / 2.0;
    move->setGoal(target);
    task.add(std::move(move));
  }
  // Detach and Remove
  {
    auto detach = std::make_unique<stages::ModifyPlanningScene>("detach_hollow");
    detach->detachObject("hollow_cylinder", hand_frame_);
    std::vector<std::string> gripper_links = {"LINK7", "LINK_TCP"};
    detach->allowCollisions("hollow_cylinder", gripper_links, true);
    detach->removeObject("hollow_cylinder");
    task.add(std::move(detach));
  }

  // Back to Home
  {
    auto home = std::make_unique<stages::MoveTo>("go_home", pipeline);
    home->setGroup(arm_group_);
    std::map<std::string, double> home_pose;
    for(int i=1; i<=7; ++i) home_pose["joint" + std::to_string(i)] = 0.0;
    home->setGoal(home_pose);
    task.add(std::move(home));
  }

  return task;
}

geometry_msgs::msg::Pose tfToPose(const tf2::Transform &tf)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = tf.getOrigin().x();
    pose.position.y = tf.getOrigin().y();
    pose.position.z = tf.getOrigin().z();
    tf2::Quaternion q = tf.getRotation();
    pose.orientation = tf2::toMsg(q);
    return pose;
}
void MTCTaskNode::setDetectorState(uint8_t state)
{
  auto req = std::make_shared<SetDetectorState::Request>();
  req->state = state;
  set_state_client_->async_send_request(req);
}
}  // namespace mtc_place


#include "mtc_place/mtc_place_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto mtc_node = std::make_shared<mtc_place::MTCTaskNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mtc_node->getNodeBaseInterface());

  executor.spin();   // 一直运行

  rclcpp::shutdown();
  return 0;
}
