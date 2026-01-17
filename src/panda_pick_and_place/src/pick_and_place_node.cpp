#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class PandaPickAndPlace : public rclcpp::Node
{
public:
    PandaPickAndPlace() : Node("panda_pick_and_place")
    {
        // 创建 MoveGroupInterface 实例，用于操作规划组
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), "main");
    }

    void run()
    {
        RCLCPP_INFO(this->get_logger(), "Hello, MoveIt! Node is running.");
        
        // 设置目标姿态
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link"; // 确保你的基坐标系名称正确
        target_pose.pose.orientation.w = 1.0;
        target_pose.pose.position.x = 0.2;
        target_pose.pose.position.y = 0.0;
        target_pose.pose.position.z = 0.2;

        move_group_->setPoseTarget(target_pose);

        // 规划和执行
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Planning successful. Executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_WARN(this->get_logger(), "Planning failed.");
        }

        // 返回默认姿势 "home"
        move_group_->setNamedTarget("home");
        move_group_->plan(plan);
        move_group_->execute(plan);

        RCLCPP_INFO(this->get_logger(), "Returned to home pose.");
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PandaPickAndPlace>();
    node->run();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}