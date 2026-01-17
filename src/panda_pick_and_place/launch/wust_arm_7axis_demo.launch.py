import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # 声明启动参数
    launch_arguments = {
        "use_fake_hardware": "true",
    }
    
    # 获取包路径
    # 使用 get_package_share_directory，它返回一个普通的字符串路径
    panda_moveit_config_share_dir = get_package_share_directory("panda_moveit_config")

    # 构建 MoveIt2 配置
    # 核心修改：使用 os.path.join 来拼接路径，避免 TypeError
    robot_description_path = os.path.join(
        panda_moveit_config_share_dir,
        "config",
        "wust_arm_7axis_description.urdf.xacro"
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="wust_arm_7axis_description", 
            package_name="panda_moveit_config"       
        )
        .robot_description(file_path=robot_description_path, mappings=launch_arguments)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    # MoveIt2 核心节点 (move_group)
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    
    # 机器人状态发布器
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # 静态TF发布器
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Rviz2 可视化节点
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )
    
    # 获取 ros2_controllers.yaml 的路径
    ros2_controllers_path = os.path.join(
        panda_moveit_config_share_dir,
        "config",
        "ros2_controllers.yaml"
    )

    # ros2_control_node 
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ('/controller_manager/robot_description', 'robot_description'),
        ],
        output="both",
    )

    # 控制器启动器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["main_controller", "-c", "/controller_manager"],
        output="screen",
    )
   
    # 你的自定义C++节点
    pick_and_place_node = Node(
        package="panda_pick_and_place",
        executable="pick_and_place_node",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([
        rviz_node,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        pick_and_place_node,
    ])