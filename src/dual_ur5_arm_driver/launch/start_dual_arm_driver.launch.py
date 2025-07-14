from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node,PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
def launch_setup(context, *args, **kwargs):
    # Load description with necessary parameters
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("dual_ur5_arm_driver"),
                    "urdf",
                    "dual_ur5_ft_gripper_base.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }
    # leftArm's arguments
    leftArm_namespace = LaunchConfiguration("leftArm_namespace")
    leftArm_kinematics = LaunchConfiguration("leftArm_kinematics")
    leftArm_controller_config_file = LaunchConfiguration("leftArm_controller_config_file")
    leftArm_ip = LaunchConfiguration("leftArm_ip")
    leftArm_reverse_port = LaunchConfiguration("leftArm_reverse_port")
    leftArm_script_sender_port = LaunchConfiguration("leftArm_script_sender_port")
    leftArm_trajectory_port = LaunchConfiguration("leftArm_trajectory_port")
    leftArm_script_command_port = LaunchConfiguration("leftArm_script_command_port")
    leftArm_update_rate_config_file = LaunchConfiguration("leftArm_update_rate_config_file")
    left_initial_joint_controller = LaunchConfiguration("left_initial_joint_controller")
    leftArm_tf_prefix = LaunchConfiguration("leftArm_tf_prefix")

    # rightArm's arguments
    rightArm_namespace = LaunchConfiguration("rightArm_namespace")
    rightArm_kinematics = LaunchConfiguration("rightArm_kinematics")
    rightArm_controller_config_file = LaunchConfiguration("rightArm_controller_config_file")
    rightArm_ip = LaunchConfiguration("rightArm_ip")
    rightArm_reverse_port = LaunchConfiguration("rightArm_reverse_port")
    rightArm_script_sender_port = LaunchConfiguration("rightArm_script_sender_port")
    rightArm_trajectory_port = LaunchConfiguration("rightArm_trajectory_port")
    rightArm_script_command_port = LaunchConfiguration("rightArm_script_command_port")
    rightArm_update_rate_config_file = LaunchConfiguration("rightArm_update_rate_config_file")
    right_initial_joint_controller = LaunchConfiguration("right_initial_joint_controller")
    rightArm_tf_prefix = LaunchConfiguration("rightArm_tf_prefix")

    # Common arguments
    dual_use_tool_communication = LaunchConfiguration("dual_use_tool_communication")
    dual_ur_type = LaunchConfiguration("dual_ur_type")
    dual_runtime_config_package = LaunchConfiguration("dual_runtime_config_package")

    # leftArm namespace group
    arm_left_driver_node = GroupAction([
        PushRosNamespace(leftArm_namespace),  # Set the 'leftArm' namespace for the group
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ur_robot_driver'), 'launch/SingleArm_ur_control.launch.py')),
            launch_arguments={
                'use_tool_communication': dual_use_tool_communication,
                "ur_type": dual_ur_type,
                "runtime_config_package":dual_runtime_config_package,
                'controllers_file': leftArm_controller_config_file,
                'robot_ip': leftArm_ip,
                'reverse_port': leftArm_reverse_port,
                'script_sender_port': leftArm_script_sender_port,
                'trajectory_port': leftArm_trajectory_port,
                'script_command_port': leftArm_script_command_port,
                'kinematics_params_file': leftArm_kinematics,
                'tf_prefix': leftArm_tf_prefix,
                'update_rate_config_file': leftArm_update_rate_config_file,
                'initial_joint_controller': left_initial_joint_controller,
                'namespace': leftArm_namespace,
            }.items()
        ),
    ])

    # rightArm namespace group
    arm_right_driver_node = GroupAction([
        PushRosNamespace(rightArm_namespace),  # Set the 'rightArm' namespace for the group
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ur_robot_driver'), 'launch/SingleArm_ur_control.launch.py')),
            launch_arguments={
                'use_tool_communication': dual_use_tool_communication,
                "ur_type": dual_ur_type,
                "runtime_config_package":dual_runtime_config_package,
                'controllers_file': rightArm_controller_config_file,
                'robot_ip': rightArm_ip,
                'reverse_port': rightArm_reverse_port,
                'script_sender_port': rightArm_script_sender_port,
                'trajectory_port': rightArm_trajectory_port,
                'script_command_port': rightArm_script_command_port,
                'kinematics_params_file': rightArm_kinematics,
                'tf_prefix': rightArm_tf_prefix,
                'update_rate_config_file': rightArm_update_rate_config_file,
                'initial_joint_controller': right_initial_joint_controller,
                'namespace': rightArm_namespace,
            }.items()
        ),
    ])
    # Convenience stuff for demo purposes
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('dual_ur5_arm_driver'), 'rviz/view_robot.rviz')],

    )
    # Robot state publisher
    robot_state_publisher_dualArm = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_dualArm',
        output='screen',
        parameters=[robot_description],
    )
    # Start the driver nodes
    nodes_to_start = [
        arm_left_driver_node,
        arm_right_driver_node,
        rviz2,
        robot_state_publisher_dualArm,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # Declare the launch arguments

    # leftArm's arguments
    declared_arguments.append(DeclareLaunchArgument('leftArm_namespace', default_value='leftArm'))
    declared_arguments.append(DeclareLaunchArgument('leftArm_kinematics', default_value=os.path.join(
        get_package_share_directory('ur_description'), 'config/ur5/default_kinematics.yaml')))
    declared_arguments.append(DeclareLaunchArgument('leftArm_controller_config_file', default_value='leftArm_controllers.yaml'))
    declared_arguments.append(DeclareLaunchArgument('leftArm_ip', default_value='10.5.0.5'))
    declared_arguments.append(DeclareLaunchArgument('leftArm_reverse_port', default_value='50001'))
    declared_arguments.append(DeclareLaunchArgument('leftArm_script_sender_port', default_value='50002'))
    declared_arguments.append(DeclareLaunchArgument('leftArm_trajectory_port', default_value='50003'))
    declared_arguments.append(DeclareLaunchArgument('leftArm_script_command_port', default_value='50004'))
    declared_arguments.append(DeclareLaunchArgument('leftArm_update_rate_config_file', default_value=PathJoinSubstitution(
        [
            FindPackageShare("dual_ur5_arm_driver"),
            "config",
            "leftArm_ur5_update_rate.yaml",
        ]
    ), description='Path to the update rate configuration file.'))
    declared_arguments.append(DeclareLaunchArgument('left_initial_joint_controller', default_value='left_ur_arm_controller'))
    declared_arguments.append(DeclareLaunchArgument('leftArm_tf_prefix', default_value='leftArm_'))

    # rightArm's arguments
    declared_arguments.append(DeclareLaunchArgument('rightArm_namespace', default_value='rightArm'))
    declared_arguments.append(DeclareLaunchArgument('rightArm_kinematics', default_value=os.path.join(
        get_package_share_directory('ur_description'), 'config/ur5/default_kinematics.yaml')))
    declared_arguments.append(DeclareLaunchArgument('rightArm_controller_config_file', default_value='rightArm_controllers.yaml'))
    declared_arguments.append(DeclareLaunchArgument('rightArm_ip', default_value='10.5.0.6'))
    declared_arguments.append(DeclareLaunchArgument('rightArm_reverse_port', default_value='60001'))
    declared_arguments.append(DeclareLaunchArgument('rightArm_script_sender_port', default_value='60002'))
    declared_arguments.append(DeclareLaunchArgument('rightArm_trajectory_port', default_value='60003'))
    declared_arguments.append(DeclareLaunchArgument('rightArm_script_command_port', default_value='60004'))
    declared_arguments.append(DeclareLaunchArgument('rightArm_update_rate_config_file', default_value=PathJoinSubstitution(
        [
            FindPackageShare("dual_ur5_arm_driver"),
            "config",
            "rightArm_ur5_update_rate.yaml",
        ]
    ), description='Path to the update rate configuration file.'))
    declared_arguments.append(DeclareLaunchArgument('right_initial_joint_controller', default_value='right_ur_arm_controller'))
    declared_arguments.append(DeclareLaunchArgument('rightArm_tf_prefix', default_value='rightArm_'))

    declared_arguments.append(DeclareLaunchArgument('dual_use_tool_communication', default_value='false'))
    declared_arguments.append(DeclareLaunchArgument('dual_ur_type', default_value='ur5'))
    declared_arguments.append(DeclareLaunchArgument('dual_runtime_config_package', default_value='dual_ur5_arm_driver'))


    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])