import os
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('wust_arm_bringup'), 'config', 'launch_params.yaml')))

node_params = os.path.join(
    get_package_share_directory('wust_arm_bringup'), 'config', 'node_params.yaml')

# 统一 MoveIt 配置加载
moveit_config = (
    MoveItConfigsBuilder("wust_arm_7axis")
    .robot_description(file_path="config/wust_arm_7axis_description.urdf.xacro")
    .robot_description_semantic(file_path="config/wust_arm_7axis_description.srdf")
    .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
    .joint_limits(file_path="config/joint_limits.yaml")
    .trajectory_execution(file_path="config/moveit_controllers.yaml")
    .robot_description_kinematics(file_path="config/kinematics.yaml")
    .planning_pipelines(pipelines=["ompl"])
    .to_moveit_configs()
)

robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="both",
    parameters=[moveit_config.robot_description],
)

move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[moveit_config.to_dict(), 
                {"capabilities": "move_group/ExecuteTaskSolutionCapability"}],
)

# 规划节点定义
mtc_place_node = Node(
    package="mtc_place",
    executable="mtc_place_node",
    output="screen",
    parameters=[
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.joint_limits,
        {"planning_plugin": "ompl_interface/OMPLPlanner"}
    ],
)


#  RViz
rviz_config = PathJoinSubstitution(
    [FindPackageShare("wust_arm_7axis_moveit_config"), "launch", LaunchConfiguration("rviz_config")]
)
rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="log",
    arguments=["-d", rviz_config],
    parameters=[
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ],
)