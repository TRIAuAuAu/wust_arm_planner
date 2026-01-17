#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void doTask();
  void setupPlanningScene();

private:
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

// 相对于 TCP 放置物体
void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "vacuum_tcp"; // 直接相对于吸盘 TCP
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = { 0.02, 0.02, 0.02 }; // 正方体

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.09;
  pose.position.y = 0.0;
  pose.position.z = -0.03; // 放在吸盘正前方
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  // 注意：在实际运行中，如果 vacuum_tcp 还未在 TF 树中出现，这里可能会警告
  psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();
  try {
    task_.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5)) {
    RCLCPP_ERROR_STREAM(LOGGER, "任务规划失败");
    return;
  }

  task_.introspection().publishSolution(*task_.solutions().front());
  
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR_STREAM(LOGGER, "任务执行失败");
    return;
  }
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("Vacuum Attach Test");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "main";
  const auto& hand_group_name = "hand"; 
  const auto& hand_frame = "vacuum_tcp";

  // 设置全局属性
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  // 1. 获取当前状态
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  auto* current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  // 定义规划器
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setStepSize(0.003);

  // 2. 自由移动到靠近物体的预备位 (Connect)
  // 这步会自动规划从当前位置到“接近阶段”起始点的路径
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick", mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

    // 3. 抓取逻辑容器
    {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick_logic");
    task.properties().exposeTo(grasp->properties(), {"group", "eef", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT);

    // 3.1 直接生成抓取位姿 (不再需要 MoveRelative)
    {
        auto stage = std::make_unique<mtc::stages::GeneratePose>("generate_pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->setMonitoredStage(current_state_ptr);

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "object"; // 目标就是物体中心
        target_pose.pose.orientation.w = 1.0;
        stage->setPose(target_pose);

        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("pick_IK", std::move(stage));
        wrapper->setIgnoreCollisions(true); // 调试用：忽略碰撞检查

        wrapper->setMaxIKSolutions(20);
        wrapper->setIKFrame(Eigen::Isometry3d::Identity(), hand_frame); 
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "eef"});
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        grasp->insert(std::move(wrapper));
    }
    
    // 3.2 简单的吸附
    {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach_object");

        stage->allowCollisions("object", "link7", true);
        // stage->allowCollisions("link3","link_fo2",true);
        stage->allowCollisions("object", arm_group_name, true); 
        stage->attachObject("object", hand_frame);
        grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
    }
    // 4. 抬起物体 (Lift)
//   // 抓取后先向上移动，避免在去放置点的路上撞到桌子
//   {
//     auto stage = std::make_unique<mtc::stages::MoveRelative>("lift_object", cartesian_planner);
//     stage->properties().set("marker_ns", "lift");
//     stage->properties().set("link", hand_frame);
//     stage->setProperty("group", arm_group_name);
//     stage->setMinMaxDistance(0.03, 0.05);

//     geometry_msgs::msg::Vector3Stamped vec;
//     vec.header.frame_id = "world"; // 沿世界坐标系 Z 轴向上
//     vec.vector.z = 1.0;
//     stage->setDirection(vec);
//     task.add(std::move(stage));
//   }

//   // 5. 移动到放置位置 (Connect)
//   {
//     auto stage = std::make_unique<mtc::stages::Connect>(
//         "move to place", mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
//     stage->setTimeout(5.0);
//     stage->properties().configureInitFrom(mtc::Stage::PARENT);
//     task.add(std::move(stage));
//   }

// //   // 6. 放置逻辑容器
//   {
//     auto place = std::make_unique<mtc::SerialContainer>("place_logic");
//     task.properties().exposeTo(place->properties(), {"group", "eef", "ik_frame"});
//     place->properties().configureInitFrom(mtc::Stage::PARENT);

//     // 6.1 生成放置位姿
//     // 注意：放置点是相对于世界或参考帧的，这里我们模拟一个目标位置
//     {
//       auto stage = std::make_unique<mtc::stages::GeneratePose>("place_pose");
//       stage->properties().configureInitFrom(mtc::Stage::PARENT);
//       stage->setMonitoredStage(current_state_ptr);

//       // 这里建议使用 "world" 作为参考，计算出放置的目标点
//       geometry_msgs::msg::PoseStamped p;
//       p.header.frame_id = "object"; 
//       p.pose.position.x = 0.03;  // 示例值：请根据实际工作空间调整
//       p.pose.position.y = 0.0; 
//       p.pose.position.z = -0.02;
//       p.pose.orientation.w = 1.0;
//       stage->setPose(p);

//       auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place_IK", std::move(stage));
//       wrapper->setMaxIKSolutions(20);
//       wrapper->setIKFrame(Eigen::Isometry3d::Identity(), hand_frame);
//       wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "eef"});
//       wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
//       place->insert(std::move(wrapper));
//     }

//     // 6.2 分离物体 (Detach)
//     {
//       auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach_object");
//       stage->detachObject("object", hand_frame);
//       place->insert(std::move(stage));
//     }
//     task.add(std::move(place));
//   }

//   // 7. 返回 Home 姿态 (所有关节设为 0)
//   {
//     auto stage = std::make_unique<mtc::stages::MoveTo>("return_home", sampling_planner);
//     stage->setProperty("group", arm_group_name);
    
//     // 直接设置关节目标值
//     std::map<std::string, double> home_joints;
//     home_joints["joint1"] = 0.0;
//     home_joints["joint2"] = 0.0;
//     home_joints["joint3"] = 0.0;
//     home_joints["joint4"] = 0.0;
//     home_joints["joint5"] = 0.0;
//     home_joints["joint6"] = 0.0;
//     home_joints["joint7"] = 0.0;
    
//     stage->setGoal(home_joints);
//     task.add(std::move(stage));
//   }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  
  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  
  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}