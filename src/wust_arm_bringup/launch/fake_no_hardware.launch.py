import os
import sys
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import TimerAction, Shutdown
bringup_path = get_package_share_directory('wust_arm_bringup')
sys.path.append(os.path.join(bringup_path, 'launch'))

def generate_launch_description():
    from common import launch_params,node_params,detector_container
    # ========== MoveIt Config ==========
    moveit_config = (
        MoveItConfigsBuilder("wust_arm_7axis")
        .robot_description(file_path="config/wust_arm_7axis_description.urdf.xacro")
        .robot_description_semantic(file_path="config/wust_arm_7axis_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # ========== Robot State Publisher ==========
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }
    #  核心节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="both",
        parameters=[moveit_config.to_dict(), 
                    move_group_capabilities],
        arguments=["--ros-args", "--log-level", "info"],
    )
    
    # ========== RViz ==========
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("wust_arm_7axis_moveit_config"), "launch", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="both",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    # ========== WUST Arm Driver ==========
    wust_arm_driver_node = Node(
        package="wust_arm_driver",
        executable="wust_arm_driver_node",
        name="wust_arm_driver",
        output="both",
        parameters=[node_params],
    )

    # ========== Static TF ==========
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    # ========== MTC ==========
    #  自定义 mtc_place_node
    mtc_place_node = Node(
        package="mtc_place",
        executable="mtc_place_node",
        output="both",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                "planning_plugin": "ompl_interface/OMPLPlanner"
            }
        ],
        ros_arguments=['--ros-args', '--log-level',
                       f'mtc_place_node:={launch_params["mtc_node_log_level"]}'],
    )
    # 延迟启动 MTC 节点，确保驱动和 MoveGroup 已就绪
    delay_mtc_node = TimerAction(
        period=1.0,
        actions=[mtc_place_node],
    )

    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        move_group_node,
        wust_arm_driver_node,
        # detector_container,
        rviz_node,
        # delay_mtc_node,
    ])
