import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    OrSubstitution,
)


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    _publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")

    # joint_limit_params = PathJoinSubstitution(
    #     [FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"]
    # )
    # kinematics_params = PathJoinSubstitution(
    #     [FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"]
    # )
    # physical_params = PathJoinSubstitution(
    #     [FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"]
    # )
    # visual_params = PathJoinSubstitution(
    #     [FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"]
    # )

    base_frame_file_path = PathJoinSubstitution(
        [FindPackageShare("dual_description"), 'config', "base_frame.yaml"]
    )    

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("dual_ur5_arm_driver"), "urdf", "dual_ur5_ft_gripper_base.urdf.xacro"]),
            ' ',
            'ur_type:=',
            'ur5',
            ' ', 
            'base_frame_file:=',
            base_frame_file_path,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "config", moveit_config_file]
            ),
        ]
    )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    #whetehr to publish robot_description_semantic or not
    publish_robot_description_semantic = {
        "publish_robot_description_semantic": _publish_robot_description_semantic
    }

    # kinematics.yaml file:
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )
    # joint_limits.yaml file:
    robot_description_planning_joint_limits = PathJoinSubstitution([
            FindPackageShare(moveit_config_package), "config", "joint_limits.yaml",
        ]
    )
    # Planning Pipelines Configuration
    planning_pipelines_config = PathJoinSubstitution([
            FindPackageShare(moveit_config_package), "config", "planning_pipelines_config.yaml",
        ]
    )
    # OMPL_Planning Configuration
    ompl_planning_config = PathJoinSubstitution([
                FindPackageShare(moveit_config_package), "config", "ompl_planning.yaml",
            ]
        )
    robot_description_planning_cartesian_limits = PathJoinSubstitution([
            FindPackageShare(moveit_config_package), "config", "cartesian_limits.yaml",
        ]
    )
    # MoveIt!2 Controllers:
    moveit_controllers = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "moveit_controllers.yaml"]
    )

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        # Execution time monitoring can be incompatible with the scaled JTC
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            publish_robot_description_semantic,

            robot_description_kinematics,
            planning_pipelines_config,
            ompl_planning_config,
            robot_description_planning_joint_limits,
            robot_description_planning_cartesian_limits,

            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
        ],
    
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", "moveit_rviz.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_pipelines_config,
            ompl_planning_config,
            robot_description_kinematics,
            robot_description_planning_joint_limits,
            robot_description_planning_cartesian_limits,
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # Servo node for realtime control
    servo_params_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "ur_servo.yaml"]
    )
    # servo_node = Node(
    #     package="moveit_servo",
    #     condition=IfCondition(launch_servo),
    #     executable="servo_node_main",
    #     parameters=[
    #         servo_params_file,
    #         robot_description,
    #         robot_description_semantic,
    #     ],
    #     output="screen",
    # )

    nodes_to_start = [move_group_node, rviz_node]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
            default_value="ur5",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Indicate whether robot is running with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="dual_arm_moveit_config",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="True",
            description="Whether to publish the SRDF description on topic /robot_description_semantic.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="dual_arm_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="dual_ur5_ft_gripper_base.srdf",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits that augment or override the values from the URDF robot_description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Servo?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
