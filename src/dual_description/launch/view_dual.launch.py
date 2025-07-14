import launch
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
import launch_ros
import os


def generate_launch_description():
    #查找并获取 ROS 2 包 dual_description 的共享路径
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="dual_description"
    ).find("dual_description")
    default_model_path = os.path.join(
        pkg_share, "urdf", "dual_ur5_ft_gripper_base.urdf.xacro"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz", "view_urdf.rviz")

    args = []
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="model",
            default_value=default_model_path,
            description="Absolute path to robot URDF file",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="rvizconfig",
            default_value=default_rviz_config_path,
            description="Absolute path to rviz config file",
        )
    )

#每一个 " " 对应于命令行中需要的一个空格，以确保命令的各个部分能够被正确识别和处理。
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            LaunchConfiguration("model"),
            " "
            "ur_type:=",
            "ur5", 
            " ", 
            "sim_gazebo:=",
            "false"
        ]
    )
    robot_description_param = {
        "robot_description": launch_ros.descriptions.ParameterValue(
            robot_description_content, value_type=str
        )
    }

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description_param],
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]

    return launch.LaunchDescription(args + nodes)
