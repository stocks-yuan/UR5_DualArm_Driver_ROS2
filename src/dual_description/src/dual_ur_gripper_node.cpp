#include "dual_description/dual_ur_gripper_node.h"

DualUrGripperNode::DualUrGripperNode(const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options)
{
    // create action client
    left_gripper_action_client_ =
        rclcpp_action::create_client<GripperCommand>(this, left_gripper_action_name_);
    right_gripper_action_client_ =
        rclcpp_action::create_client<GripperCommand>(this, right_gripper_action_name_);
    send_goal_options_.goal_response_callback =
        std::bind(&DualUrGripperNode::goal_response_callback, this, std::placeholders::_1);
    send_goal_options_.feedback_callback =
        std::bind(&DualUrGripperNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options_.result_callback =
        std::bind(&DualUrGripperNode::result_callback, this, std::placeholders::_1);
    // wait for action server to come up
    RCLCPP_INFO(this->get_logger(), "Waiting for action server");
    left_gripper_action_client_->wait_for_action_server(std::chrono::seconds(5));
    right_gripper_action_client_->wait_for_action_server(std::chrono::seconds(5));
    if (!left_gripper_action_client_->action_server_is_ready() ||
        !right_gripper_action_client_->action_server_is_ready())
    {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        throw std::runtime_error("Action server not available");
    }
}

void DualUrGripperNode::init()
{
    // moveit
    both_move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), both_planning_group_name_);
    left_move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), left_planning_group_name_);
    right_move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), right_planning_group_name_);

    // for fast demo
    both_move_group_->setPlanningTime(20.0); // 设置规划超时时间为10秒
    both_move_group_->allowReplanning(true);

    both_move_group_->setMaxAccelerationScalingFactor(0.3);
    both_move_group_->setMaxVelocityScalingFactor(0.3);

    both_move_group_->setPlanningPipelineId("ompl"); // refer to the planning_pipeline_config.yaml
    both_move_group_->setPlannerId("RRTConnectkConfigDefault");

    left_move_group_->allowReplanning(true);
    left_move_group_->setPlanningPipelineId("ompl"); // refer to the planning_pipeline_config.yaml
    left_move_group_->setPlannerId("RRTConnectkConfigDefault");
    left_move_group_->setPlanningTime(20.0); // 设置规划超时时间为10秒
    left_move_group_->allowReplanning(true);

    right_move_group_->allowReplanning(true);
    right_move_group_->setPlanningPipelineId("ompl"); // refer to the planning_pipeline_config.yaml
    right_move_group_->setPlannerId("RRTConnectkConfigDefault");
    right_move_group_->setPlanningTime(20.0); // 设置规划超时时间为10秒
    right_move_group_->allowReplanning(true);
}

bool DualUrGripperNode::go_to_ready_position()
{
    std::vector<double> both_ready_joint = {
        1.09624,
        -2.44259,
        2.22879,
        -0.591143,
        1.74742,
        0.523774,
        0.268083,
        -1.50692,
        -2.18829,
        4.33069,
        -1.47899,
        -1.29364,
    };
    both_move_group_->setJointValueTarget(both_ready_joint);

    const int max_retries = 5;
    int retry_count = 0;
    bool success = false;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    while (retry_count < max_retries)
    {
        RCLCPP_INFO(this->get_logger(), "Planning attempt %d", retry_count + 1);
        success = (both_move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            both_move_group_->execute(my_plan);
            RCLCPP_INFO(this->get_logger(), "Go to ready position successfully on attempt %d", retry_count + 1);
            break;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Go to ready position failed on attempt %d", retry_count + 1);
        }

        retry_count++;
    }

    if (!success)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to go to ready position after %d attempts", max_retries);
    }

    return success;
}

bool DualUrGripperNode::go_to_ready_position_stage()
{
    std::vector<std::vector<double>> staged_positions = {
        {1.09624, -2.44259, 2.22879, -0.591143, 1.74742, 0.523774,
         0.268083, -1.50692, -2.18829, 4.33069, -1.47899, -1.29364}, // 最终目标位置
        // 添加中间阶段的关节位置
        {1.0, -2.0, 2.0, -0.5, 1.5, 0.5, 0.2, -1.5, -2.0, 4.3, -1.5, -1.2},
        {0.5, -1.0, 1.0, -0.2, 0.7, 0.2, 0.1, -1.0, -1.0, 3.0, -0.5, -0.6}};

    both_move_group_->setPlanningTime(20.0);             // 增加规划时间到20秒
    both_move_group_->setPlannerId("RRTstar");           // 尝试使用不同的规划器
    both_move_group_->setGoalPositionTolerance(0.01);    // 设置位置容忍度
    both_move_group_->setGoalOrientationTolerance(0.01); // 设置姿态容忍度

    const int max_retries = 5; // 设置最大重试次数
    bool success = true;

    for (const auto &position : staged_positions)
    {
        both_move_group_->setJointValueTarget(position);
        int retry_count = 0;
        bool stage_success = false;

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        while (retry_count < max_retries)
        {
            RCLCPP_INFO(this->get_logger(), "Planning attempt %d for staged position", retry_count + 1);
            stage_success = (both_move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (stage_success)
            {
                both_move_group_->execute(my_plan);
                RCLCPP_INFO(this->get_logger(), "Reached staged position successfully on attempt %d", retry_count + 1);
                break;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to reach staged position on attempt %d", retry_count + 1);
            }

            retry_count++;
        }

        if (!stage_success)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to reach staged position after %d attempts", max_retries);
            success = false;
            break;
        }
    }

    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "Successfully reached the final ready position");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to reach the final ready position");
    }

    return success;
}

bool DualUrGripperNode::plan_and_execute(
    const geometry_msgs::msg::Pose &left_target_pose,
    const geometry_msgs::msg::Pose &right_target_pose,
    const std::string &left_reference_frame,
    const std::string &right_reference_frame,
    const std::string &left_end_effector_link,
    const std::string &right_end_effector_link)
{
    const int max_retries = 15;
    int retry_count = 0;
    bool success = false;

    while (retry_count < max_retries)
    {
        auto const left_target_pose_stamped = [&left_target_pose, &left_reference_frame]
        {
            geometry_msgs::msg::PoseStamped msg;
            msg.header.frame_id = left_reference_frame;
            msg.pose = left_target_pose;
            return msg;
        }();

        auto const right_target_pose_stamped = [&right_target_pose, &right_reference_frame]
        {
            geometry_msgs::msg::PoseStamped msg;
            msg.header.frame_id = right_reference_frame;
            msg.pose = right_target_pose;
            return msg;
        }();

        left_target_joint_.clear();
        right_target_joint_.clear();
        both_target_joint_.clear();

        left_move_group_->setStartStateToCurrentState();
        left_move_group_->setJointValueTarget(left_target_pose_stamped, left_end_effector_link);
        left_move_group_->getJointValueTarget(left_target_joint_); // use ik to get joint target from target pose

        right_move_group_->setStartStateToCurrentState();
        right_move_group_->setJointValueTarget(right_target_pose_stamped, right_end_effector_link);
        right_move_group_->getJointValueTarget(right_target_joint_); // use ik to get joint target from target pose

        // combine both target joint
        both_target_joint_.insert(both_target_joint_.end(), left_target_joint_.begin(), left_target_joint_.end());
        both_target_joint_.insert(both_target_joint_.end(), right_target_joint_.begin(), right_target_joint_.end());

        // set the target positions for both manipulators in joint space
        both_move_group_->setStartStateToCurrentState();
        both_move_group_->setJointValueTarget(both_target_joint_);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        success = (both_move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
            both_move_group_->execute(my_plan);
            RCLCPP_INFO(this->get_logger(), "Plan and execute succeeded on attempt %d", retry_count + 1);
            break;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Plan and execute failed on attempt %d", retry_count + 1);
        }
        retry_count++;
    }

    if (!success)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to plan and execute after %d attempts", max_retries);
    }
    return success;
}

bool DualUrGripperNode::plan_single_arm(
    bool left,
    const geometry_msgs::msg::Pose &target_pose,
    const std::string &reference_frame,
    const std::string &end_effector_link,
    moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    const int max_retries = 15;
    int retry_count = 0;
    bool success = false;

    auto move_group = left ? left_move_group_ : right_move_group_;

    auto const target_pose_stamped = [&target_pose, &reference_frame]
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = reference_frame;
        msg.pose = target_pose;
        return msg;
    }();

    while (retry_count < max_retries)
    {
        move_group->setStartStateToCurrentState();
        move_group->setJointValueTarget(target_pose_stamped, end_effector_link);

        success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Plan single arm succeeded on attempt %d", retry_count + 1);
            break;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Plan single arm failed on attempt %d", retry_count + 1);
        }
        retry_count++;
    }

    if (!success)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to plan single arm after %d attempts", max_retries);
    }
    return success;
}

void DualUrGripperNode::execute_single_arm(
    bool left,
    moveit::planning_interface::MoveGroupInterface::Plan &single_plan,
    bool success)
{

    auto move_group = left ? left_move_group_ : right_move_group_;

    if (success)
    {
        move_group->execute(single_plan);
        RCLCPP_INFO(this->get_logger(), "Plan and execute successfully");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Plan and execute failed");
    }
}

bool DualUrGripperNode::grasp(bool left, double gripper_position)
{
    auto gripper_goal_msg = GripperCommand::Goal();
    gripper_goal_msg.command.position = gripper_position;
    gripper_goal_msg.command.max_effort = -1.0; // do not limit the effort

    auto gripper_action_client = left ? left_gripper_action_client_ : right_gripper_action_client_;
    if (!gripper_action_client->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Sending gripper goal");
    gripper_action_client->async_send_goal(gripper_goal_msg, send_goal_options_);
    return true;
}

bool DualUrGripperNode::left_grasp(double gripper_position)
{
    return grasp(true, gripper_position);
}

bool DualUrGripperNode::right_grasp(double gripper_position)
{
    return grasp(false, gripper_position);
}

void DualUrGripperNode::left_attach_object(moveit_msgs::msg::CollisionObject &obj,
                   const std::string &left_end_effector_link)
{
    // 修改参考坐标系到被依附link
    obj.header.frame_id = left_end_effector_link;
    obj.operation = obj.ADD;

    // 创建AttachedCollisionObject
    moveit_msgs::msg::AttachedCollisionObject attached_obj;
    attached_obj.link_name = left_end_effector_link;
    attached_obj.object = obj; // 拷贝
    //planning_scene.robot_state.attached_collision_objects.push_back(attached_obj);
    planning_scene_interface_.applyAttachedCollisionObject(attached_obj);
}

void DualUrGripperNode::goal_response_callback(const GoalHandleGripperCommand::SharedPtr &goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}
void DualUrGripperNode::feedback_callback(GoalHandleGripperCommand::SharedPtr, const std::shared_ptr<const GripperCommand::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Got Feedback: Current position is %f", feedback->position);
}
void DualUrGripperNode::result_callback(const GoalHandleGripperCommand::WrappedResult &result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Goal is completed, current position is %f", result.result->position);
}
