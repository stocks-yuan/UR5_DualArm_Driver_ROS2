#ifndef DUAL_UR_ROBOTIQ_RS_DESCRIPTION_NODE_H_
#define DUAL_UR_ROBOTIQ_RS_DESCRIPTION_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <chrono>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class DualUrGripperNode : public rclcpp::Node
{
public:
    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

     /**
     * @brief Construct a new Dual Ur Gripper Node object
     * 
     * @param options 
     */
    explicit DualUrGripperNode(const std::string & node_name,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief initialize move group in init function
     * 
    *  MoveGroupInterface needs the shared point of the Node, but shared_ptr won't be created until after the constructor returns
    *  so we need to create the move group interface in a separate function
    * @ref https://robotics.stackexchange.com/questions/96027/getting-a-nodesharedptr-from-this
     */
    void init();

    bool go_to_ready_position();
    bool go_to_ready_position_stage();

    bool plan_and_execute(
        const geometry_msgs::msg::Pose & left_target_pose, 
        const geometry_msgs::msg::Pose & right_target_pose, 
        const std::string & left_reference_frame = "left_base_link", 
        const std::string & right_reference_frame = "right_base_link", 
        const std::string & left_end_effector_link = "",
        const std::string & right_end_effector_link = "");

    bool plan_single_arm(
        bool left,
        const geometry_msgs::msg::Pose & target_pose, 
        const std::string & reference_frame,
        const std::string & end_effector_link, 
        moveit::planning_interface::MoveGroupInterface::Plan & plan
    );

    void execute_single_arm(
        bool left,
        moveit::planning_interface::MoveGroupInterface::Plan & single_plan,
        bool success);
        
    void add_collision_objects(std::vector<moveit_msgs::msg::CollisionObject> & collision_objects){
        planning_scene_interface_.addCollisionObjects(collision_objects);
    }

    void left_attach_object(moveit_msgs::msg::CollisionObject &obj,
                   const std::string &left_end_effector_link);

    

    /**
     * @brief grasp the object
     * 
     * @param gripper_position target position for the robotiq gripper
     */
    bool left_grasp(double gripper_position);
    bool right_grasp(double gripper_position);
    // Helper functions

    /**
     * @brief 链式构造
     */
    /**
     * @brief Set the both planning group name object
     * 
     * @param planning_group_name 
     * @return DualUrGripperNode& 
     */
    DualUrGripperNode & set_both_planning_group_name(const std::string & planning_group_name){
        both_planning_group_name_ = planning_group_name;
        return *this;
    }
    /**
     * @brief Set the left planning group name object
     * 
     * @param planning_group_name 
     * @return DualUrGripperNode& 
     */
    DualUrGripperNode & set_left_planning_group_name(const std::string & planning_group_name){
        left_planning_group_name_ = planning_group_name;
        return *this;
    }
    DualUrGripperNode & set_left_gripper_action_name(const std::string & gripper_action_name){
        left_gripper_action_name_ = gripper_action_name;
        return *this;
    }
    /**
     * @brief Set the right planning group name object
     * 
     * @param planning_group_name 
     * @return DualUrGripperNode& 
     */
    DualUrGripperNode & set_right_planning_group_name(const std::string & planning_group_name){
        right_planning_group_name_ = planning_group_name;
        return *this;
    }

    DualUrGripperNode & set_right_gripper_action_name(const std::string & gripper_action_name){
        right_gripper_action_name_ = gripper_action_name;
        return *this;
    }


    moveit::planning_interface::MoveGroupInterfacePtr get_both_move_group(){
        return both_move_group_;
    }
    moveit::planning_interface::MoveGroupInterfacePtr get_left_move_group(){
        return left_move_group_;
    }
    moveit::planning_interface::MoveGroupInterfacePtr get_right_move_group(){
        return right_move_group_;
    }
    

private:
    // moveit
    std::string both_planning_group_name_ = "both_manipulators";
    std::string left_planning_group_name_ = "left_ur_manipulator";
    std::string right_planning_group_name_ = "right_ur_manipulator";
    moveit::planning_interface::MoveGroupInterfacePtr both_move_group_;
    moveit::planning_interface::MoveGroupInterfacePtr left_move_group_;
    moveit::planning_interface::MoveGroupInterfacePtr right_move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    std::vector<double> left_target_joint_;
    std::vector<double> right_target_joint_;
    std::vector<double> both_target_joint_;

    // robotiq gripper
    std::string left_gripper_action_name_ = "/left_gripper_controller/gripper_cmd";
    std::string right_gripper_action_name_ = "/right_gripper_controller/gripper_cmd";
    rclcpp_action::Client<GripperCommand>::SharedPtr left_gripper_action_client_;
    rclcpp_action::Client<GripperCommand>::SharedPtr right_gripper_action_client_;
    void goal_response_callback(const GoalHandleGripperCommand::SharedPtr & goal_handle);
    void feedback_callback(GoalHandleGripperCommand::SharedPtr, const std::shared_ptr<const GripperCommand::Feedback> feedback);
    void result_callback(const GoalHandleGripperCommand::WrappedResult & result);
    rclcpp_action::Client<GripperCommand>::SendGoalOptions send_goal_options_;
    
    bool grasp(bool left, double gripper_position);
};




#endif
