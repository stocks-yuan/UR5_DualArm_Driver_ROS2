#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>

#include <pluginlib/class_loader.hpp>  // 加入 pluginlib 头文件
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/msg/pose.h>
#include <thread>
#include <Eigen/Geometry>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/trajectory_tools.h> // 包含 scaleRobotTrajectoryTiming 所需的头文件
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dual_arm_move_group_demo");

int main(int argc, char** argv)
{
    // 初始化 ROS 2 客户端库
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("dual_move_group_interface",node_options);

    // 创建 ROS 2 执行器和线程,监控机器人的状态
    rclcpp::executors::MultiThreadedExecutor executor; // 使用多线程执行器
    executor.add_node(move_group_node);
    std::thread spin_thread([&executor]() { executor.spin();});

    // 定义左右手臂的 MoveGroup 名称
    static const std::string LEFT_ARM_GROUP = "left_ur_manipulator";  
    static const std::string RIGHT_ARM_GROUP = "right_ur_manipulator"; 
    static const std::string DUAL_ARM_GROUP = "both_manipulators";

   
    // 设置机器人模型加载器，使用机器人描述文件"robot_description"加载模型
    robot_model_loader::RobotModelLoader robot_model_loader(move_group_node,"robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    // 创建PlanningScene对象，管理机器人状态和场景
    planning_scene::PlanningScene planning_scene(kinematic_model);
    // 定义场景接口
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));


    const moveit::core::JointModelGroup* joint_model_group_left = kinematic_state->getJointModelGroup(LEFT_ARM_GROUP);
    const moveit::core::JointModelGroup* joint_model_group_right = kinematic_state->getJointModelGroup(RIGHT_ARM_GROUP);
    const moveit::core::JointModelGroup* joint_model_group_dual = kinematic_state->getJointModelGroup(DUAL_ARM_GROUP);
    
    // 分别为左右手臂创建MoveGroupInterface对象
    moveit::planning_interface::MoveGroupInterface left_arm_group(move_group_node, LEFT_ARM_GROUP);
    moveit::planning_interface::MoveGroupInterface right_arm_group(move_group_node, RIGHT_ARM_GROUP);
    moveit::planning_interface::MoveGroupInterface dual_arm_group(move_group_node, DUAL_ARM_GROUP);


    // 设置机器人的最大速度和加速度
    left_arm_group.setMaxVelocityScalingFactor(0.5);
    left_arm_group.setMaxAccelerationScalingFactor(0.5);
    right_arm_group.setMaxVelocityScalingFactor(0.5);
    right_arm_group.setMaxAccelerationScalingFactor(0.5);

    // 启动状态监控
    left_arm_group.startStateMonitor(1.0);
    right_arm_group.startStateMonitor(1.0);
    dual_arm_group.startStateMonitor(1.0);

    // 设置规划时间为10秒
    left_arm_group.setPlanningTime(10.0);
    right_arm_group.setPlanningTime(10.0);
    dual_arm_group.setPlanningTime(10.0);

    left_arm_group.setWorkspace(-0.8, -0.8, 0.1, 0.0, 0.0, 1.3);
    right_arm_group.setWorkspace(0.0, -0.8, 0.1, 0.8, 0.0, 1.3);

    // 打印机器人的关节名称
    RCLCPP_INFO(LOGGER, "Left Arm Joint Names:"); 
    for (const auto& joint_name : joint_model_group_left->getActiveJointModelNames()) {
        RCLCPP_INFO(LOGGER, "  - %s", joint_name.c_str());
    }

    RCLCPP_INFO(LOGGER, "Right Arm Joint Names:");
    for (const auto& joint_name : joint_model_group_right->getActiveJointModelNames()) {
        RCLCPP_INFO(LOGGER, "  - %s", joint_name.c_str());
    }

    RCLCPP_INFO(LOGGER, "Dual Arm Joint Names:");
    for (const auto& joint_name : joint_model_group_dual->getActiveJointModelNames()) {
        RCLCPP_INFO(LOGGER, "  - %s", joint_name.c_str());
    }

    // 获取左右手臂的参考坐标系和末端执行器
    RCLCPP_INFO(LOGGER, "Left Arm Planning frame: %s", left_arm_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "Right Arm Planning frame: %s", right_arm_group.getPlanningFrame().c_str());

    RCLCPP_INFO(LOGGER, "Left Arm End effector link: %s", left_arm_group.getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Right Arm End effector link: %s", right_arm_group.getEndEffectorLink().c_str());

    // 打印左右手臂的当前位姿
    geometry_msgs::msg::Pose left_current_pose = dual_arm_group.getCurrentPose("left_tool0").pose;
    RCLCPP_INFO(LOGGER, "Left Arm Current Pose:");
    RCLCPP_INFO(LOGGER, "  - Position: (%.2f, %.2f, %.2f)",
                left_current_pose.position.x,
                left_current_pose.position.y,
                left_current_pose.position.z);

    geometry_msgs::msg::Pose right_current_pose = dual_arm_group.getCurrentPose("right_tool0").pose;
    RCLCPP_INFO(LOGGER, "Right Arm Current Pose:");
    RCLCPP_INFO(LOGGER, "  - Position: (%.2f, %.2f, %.2f)",
                right_current_pose.position.x,
                right_current_pose.position.y,
                right_current_pose.position.z);


    namespace rvt = rviz_visual_tools; // 命名空间别名
    // 创建可视化工具实例
    moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "world", 
                                                "dual_arm_visual_tools",dual_arm_group.getRobotModel());
    visual_tools.deleteAllMarkers(); // 删除所有标记

    /* 远程控制是一种可以让用户通过按钮和键盘快捷键逐步执行高层脚本的工具 */
    visual_tools.loadRemoteControl(); // 加载远程控制

    // RViz提供了许多类型的标记
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity(); // 创建文本位置的单位变换
    text_pose.translation().z() = 1.5; // 设置文本的Z轴位置
    visual_tools.publishText(text_pose, "Dual_Arm_MoveGroup_Demo", rvt::WHITE, rvt::XLARGE); // 发布文本标记

    // 批量发布用于减少大量可视化时发送给RViz的消息数量
    visual_tools.trigger(); // 触发可视化工具

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to set Cartesian target poses");

    // 定义左臂的路径点
    std::vector<geometry_msgs::msg::Pose> waypoints_left;
    // 获取左臂当前位姿
    geometry_msgs::msg::Pose start_pose_left = left_arm_group.getCurrentPose("left_tool0").pose;
    // 将当前位姿作为第一个路径点
    waypoints_left.push_back(start_pose_left);

    // 定义左臂的目标姿态
    geometry_msgs::msg::Pose target_pose_left = start_pose_left;
    // 设置左臂的方向
    tf2::Quaternion left_orientation;
    double left_roll = 0.0;
    double left_pitch = -M_PI / 2;
    double left_yaw = 0.0;
    left_orientation.setRPY(left_roll, left_pitch, left_yaw);
    geometry_msgs::msg::Quaternion left_quat = tf2::toMsg(left_orientation);

    // 设置左臂的路径点并设置方向
    target_pose_left.position.x += 0.08;
    target_pose_left.orientation = left_quat;  // 设置方向
    waypoints_left.push_back(target_pose_left);

    target_pose_left.position.y -= 0.09;
    target_pose_left.orientation = left_quat;
    waypoints_left.push_back(target_pose_left);

    // 第四个目标位姿
    target_pose_left.position.z += 0.1;
    target_pose_left.orientation = left_quat;
    waypoints_left.push_back(target_pose_left);


    // 定义右臂的路径点
    std::vector<geometry_msgs::msg::Pose> waypoints_right;
    // 获取右臂当前位姿
    geometry_msgs::msg::Pose start_pose_right = right_arm_group.getCurrentPose("right_tool0").pose;
    // 将当前位姿作为第一个路径点
    waypoints_right.push_back(start_pose_right);
    // 定义右臂的目标姿态
    geometry_msgs::msg::Pose target_pose_right = start_pose_right;
    // 设置右臂的方向
    tf2::Quaternion right_orientation;
    double right_roll = 0.0;
    double right_pitch = M_PI/2;
    double right_yaw = 0.0;
    right_orientation.setRPY(right_roll, right_pitch, right_yaw);
    geometry_msgs::msg::Quaternion right_quat = tf2::toMsg(right_orientation);

    // 第一个目标位姿
    target_pose_right.position.x -= 0.33;
    target_pose_right.orientation = right_quat;
    waypoints_right.push_back(target_pose_right);

    // 第二个目标位姿
    target_pose_right.position.y -= 0.08;
    target_pose_right.orientation = right_quat;
    waypoints_right.push_back(target_pose_right);

    // 第三个目标位姿
    target_pose_right.position.z += 0.27;
    target_pose_right.orientation = right_quat;
    waypoints_right.push_back(target_pose_right);

    // 为左臂规划笛卡尔路径
    moveit_msgs::msg::RobotTrajectory trajectory_left;
    double fraction_left = left_arm_group.computeCartesianPath(
        waypoints_left, // 路径点
        0.01,           // 步长，控制路径的细分程度
        0.0,            // 跳跃阈值，0.0表示禁用跳跃检测
        trajectory_left
    );

    RCLCPP_INFO(LOGGER, "左臂笛卡尔路径规划完成，完成度：%.2f%%", fraction_left * 100.0);

    // 为右臂规划笛卡尔路径
    moveit_msgs::msg::RobotTrajectory trajectory_right;
    double fraction_right = right_arm_group.computeCartesianPath(
        waypoints_right,
        0.01,
        0.0,
        trajectory_right
    );

    RCLCPP_INFO(LOGGER, "右臂笛卡尔路径规划完成，完成度：%.2f%%", fraction_right * 100.0);

    // 可视化左臂路径点
    for (size_t i = 0; i < waypoints_left.size(); ++i) {
        visual_tools.publishAxisLabeled(waypoints_left[i], "Left_WP_" + std::to_string(i));
    }

    // 可视化右臂路径点
    for (size_t i = 0; i < waypoints_right.size(); ++i) {
        visual_tools.publishAxisLabeled(waypoints_right[i], "Right_WP_" + std::to_string(i));
    }

    // 可视化路径
    visual_tools.publishPath(waypoints_left, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishPath(waypoints_right, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.trigger();

    //将轨迹转换为 RobotTrajectory 对象
    robot_trajectory::RobotTrajectory rt_left(kinematic_model, LEFT_ARM_GROUP);
    robot_trajectory::RobotTrajectory rt_right(kinematic_model, RIGHT_ARM_GROUP);

    rt_left.setRobotTrajectoryMsg(*kinematic_state, trajectory_left);
    rt_right.setRobotTrajectoryMsg(*kinematic_state, trajectory_right);


    // 获取左右手臂轨迹的总时长
    double duration_left = rt_left.getWayPointDurationFromStart(rt_left.getWayPointCount() - 1);
    double duration_right = rt_right.getWayPointDurationFromStart(rt_right.getWayPointCount() - 1);
    double max_duration = std::max(duration_left, duration_right);

    // 确定统一的时间步长
    double time_interval = 0.01; // 时间步长，可以根据需要调整
    size_t num_points = static_cast<size_t>(std::ceil(max_duration / time_interval)) + 1;

    // 在统一的时间点上获取左右手臂的插值状态并合并
    robot_trajectory::RobotTrajectory rt_dual(kinematic_model, DUAL_ARM_GROUP);

    for (size_t i = 0; i < num_points; ++i) {
        double sample_time = i * time_interval;
        if (sample_time > max_duration) sample_time = max_duration;

        // 创建新的机器人状态
        moveit::core::RobotStatePtr dual_state(new moveit::core::RobotState(kinematic_model));

        // 获取左臂的插值状态
     
        // 创建一个新的 RobotStatePtr，用于存储插值状态
        moveit::core::RobotStatePtr interpolated_state_left(new moveit::core::RobotState(kinematic_model));

        if (sample_time <= duration_left) {
            // 获取在指定时间的机器人状态
            rt_left.getStateAtDurationFromStart(sample_time, interpolated_state_left);
        } else {
            // 获取左臂轨迹的最后一个状态，并复制到 interpolated_state_left 中
            const moveit::core::RobotState& last_state_left = rt_left.getLastWayPoint();
            *interpolated_state_left = last_state_left;
        }
        // 获取右臂的插值状态
        // 创建一个新的 RobotStatePtr，用于存储插值状态
        moveit::core::RobotStatePtr interpolated_state_right(new moveit::core::RobotState(kinematic_model));

        if (sample_time <= duration_right) {
            // 获取在指定时间的机器人状态
            rt_right.getStateAtDurationFromStart(sample_time, interpolated_state_right);
        } else {
            // 获取右臂轨迹的最后一个状态，并复制到 interpolated_state_right 中
            const moveit::core::RobotState& last_state_right = rt_right.getLastWayPoint();
            *interpolated_state_right = last_state_right;
        }

        // 合并左右手臂状态
        std::vector<double> joint_values_left;
        interpolated_state_left->copyJointGroupPositions(joint_model_group_left, joint_values_left);
        dual_state->setJointGroupPositions(joint_model_group_left, joint_values_left);

        std::vector<double> joint_values_right;
        interpolated_state_right->copyJointGroupPositions(joint_model_group_right, joint_values_right);
        dual_state->setJointGroupPositions(joint_model_group_right, joint_values_right);

        // 设置时间间隔
        double time_from_previous = (i == 0) ? 0.0 : time_interval;
        rt_dual.addSuffixWayPoint(dual_state, time_from_previous);
    }

    // 对合并后的轨迹进行时间参数化（可选）
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool success_iptp_1 = iptp.computeTimeStamps(rt_dual);

    if (!success_iptp_1) {
        RCLCPP_ERROR(LOGGER, "双臂轨迹时间参数化失败！");
        return -1;
    }

    // 转换为消息并执行
    moveit_msgs::msg::RobotTrajectory trajectory_dual;
    rt_dual.getRobotTrajectoryMsg(trajectory_dual);

    moveit::planning_interface::MoveGroupInterface::Plan plan_dual;
    plan_dual.trajectory_ = trajectory_dual;

    visual_tools.publishTrajectoryLine(plan_dual.trajectory_, joint_model_group_dual);
    visual_tools.trigger();

    visual_tools.prompt("按下 'next' 开始执行第1段同步的双臂轨迹");

    moveit::core::MoveItErrorCode execution_result = dual_arm_group.execute(plan_dual);

    if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(LOGGER, "双臂轨迹执行成功！");
    } else {
        RCLCPP_ERROR(LOGGER, "双臂轨迹执行失败！");
    }

    // 结束并关闭节点
    rclcpp::shutdown();
    return 0;
}

