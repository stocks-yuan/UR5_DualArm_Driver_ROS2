from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
import os

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'runtime_config_package',
            default_value='dual_description',
            description='Package with the controller\'s configuration in "config" folder. \
                         Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='dual_description',
            description='Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='dual_ur5_ft_gripper_base.urdf.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint names, useful for multi-robot setup. \
                         If changed than also joint names in the controllers \
                         configuration have to be updated. Expected format "<prefix>/"',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config_file', 
            default_value='dual.rviz',
            description='Rviz file'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'base_frame_file',
            default_value='base_frame.yaml',
            description='Configuration file of robot base frame wrt World.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        )
    )

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    prefix = LaunchConfiguration('prefix')
    start_rviz = LaunchConfiguration('start_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    base_frame_file = LaunchConfiguration('base_frame_file')
    namespace = LaunchConfiguration('namespace')
    use_sim = LaunchConfiguration('use_sim')

    rviz_config_file_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'rviz', rviz_config_file]
    )
    base_frame_file_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config', base_frame_file]
    )

    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'urdf', description_file]
            ),
            ' ',
            'ur_type:=',
            'ur5',
            ' ', 
            'base_frame_file:=',
            base_frame_file_path,
        ]
    )

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
        # it is the save way to wrap the xacro output
        # ref: https://answers.ros.org/question/417369/caught-exception-in-launch-see-debug-for-traceback-unable-to-parse-the-value-of-parameter-robot_description-as-yaml/
    }

     
    # Get SRDF via xacro
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "srdf", "dual_ur5_ft_gripper_base.srdf.xacro"]
            ),
        ]
    )

    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)
    }

    # Get planning parameters
    # Kinematics.yaml file:
    # /**:ros__parameters:robot_description_planning等价于
        # kinematics_yaml = load_yaml("ros2srrc_robots", CONFIGURATION["rob"] + "/config/kinematics.yaml")
        # robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(description_package), "moveit2", "kinematics.yaml"]
    )

    # joint_limits.yaml file:
    robot_description_planning_joint_limits = PathJoinSubstitution([
            FindPackageShare(description_package), "moveit2", "joint_limits.yaml",
        ]
    )

    planning_pipelines_config = PathJoinSubstitution([
            FindPackageShare(description_package), "moveit2", "planning_pipelines_config.yaml",
        ]
    )

    ompl_planning_config = PathJoinSubstitution([
            FindPackageShare(description_package), "moveit2", "ompl_planning.yaml",
        ]
    )

    robot_description_planning_cartesian_limits = PathJoinSubstitution([
            FindPackageShare(description_package), "moveit2", "cartesian_limits.yaml",
        ]
    )



    # MoveIt!2 Controllers:
    moveit_controllers = PathJoinSubstitution(
        [FindPackageShare(description_package),
            "moveit2", "moveit_controllers.yaml"]
    )

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description":True,
        "publish_robot_description_semantic":True,
    }

    move_group_capabilities = {
    "capabilities": """move_group/ExecuteTaskSolutionCapability"""
    }

    # move_group_capabilities = {
    # "capabilities": """pilz_industrial_motion_planner/MoveGroupSequenceAction \
    #     pilz_industrial_motion_planner/MoveGroupSequenceService \
    #     move_group/ExecuteTaskSolutionCapability"""
    # }


    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # MoveGroup Node:
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,

            planning_pipelines_config,
            ompl_planning_config,
            robot_description_planning_cartesian_limits,
            robot_description_planning_joint_limits,
            
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {"use_sim_time": True},
            warehouse_ros_config,
        ],
    )

    # RVIZ:
    rviz_node_full = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file_path],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipelines_config,
            ompl_planning_config,
            robot_description_planning_cartesian_limits,
            robot_description_planning_joint_limits,
            
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {"use_sim_time": True},
            warehouse_ros_config,
        ],
        condition=IfCondition(start_rviz),
    )

    nodes = [
        run_move_group_node,
        rviz_node_full,
    ]

    return LaunchDescription(declared_arguments + nodes)
