#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

#include <thread>
#include <fstream>
#include <vector>
#include <map>
#include <chrono>
#include <matplot/matplot.h>

// 命名空间定义
const std::string LEFT_NS = "/leftArm";
const std::string RIGHT_NS = "/rightArm";
const std::string LEFT_CONTROLLER = LEFT_NS + "/left_ur_arm_controller";
const std::string RIGHT_CONTROLLER = RIGHT_NS + "/right_ur_arm_controller";
const std::string LEFT_ACTION = LEFT_CONTROLLER + "/follow_joint_trajectory";
const std::string RIGHT_ACTION = RIGHT_CONTROLLER + "/follow_joint_trajectory";
const std::string JOINT_STATES_TOPIC = "joint_states";

const std::vector<std::string> left_joints = {"leftArm_shoulder_pan_joint", "leftArm_shoulder_lift_joint",
                                              "leftArm_elbow_joint", "leftArm_wrist_1_joint",
                                              "leftArm_wrist_2_joint", "leftArm_wrist_3_joint"};

const std::vector<std::string> right_joints = {"rightArm_shoulder_pan_joint", "rightArm_shoulder_lift_joint",
                                               "rightArm_elbow_joint", "rightArm_wrist_1_joint",
                                               "rightArm_wrist_2_joint", "rightArm_wrist_3_joint"};

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class DualArmExecutor : public rclcpp::Node
{
public:
  DualArmExecutor() : Node("dual_arm_executor")
  {
    // 初始化Action客户端
    left_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, LEFT_ACTION);
    right_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, RIGHT_ACTION);
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        JOINT_STATES_TOPIC, rclcpp::SystemDefaultsQoS(),
        std::bind(&DualArmExecutor::joint_state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "正在连接左臂控制器: %s", LEFT_ACTION.c_str());
    RCLCPP_INFO(this->get_logger(), "正在连接右臂控制器: %s", RIGHT_ACTION.c_str());

    init_joint_data_containers();
    this->config = load_yaml_file();
  }

  void execute_trajectories()
  {
    // 清空之前记录的数据
    clear_recorded_data();

    // 步骤1：等待控制器连接
    if (!wait_for_controllers(10))
      return;

    Init_dual_arm_pose();
    // 步骤2：创建轨迹目标
    auto left_goal = create_goal(LEFT_NS);
    auto right_goal = create_goal(RIGHT_NS);

    // 步骤3：设置同步时间戳
    auto now = this->now();
    left_goal.trajectory.header.stamp = now;
    right_goal.trajectory.header.stamp = now;

    // 记录开始时间
    start_time_ = std::chrono::steady_clock::now();
    recording_active_ = true;

    // 步骤4：异步发送目标
    auto left_future = left_client_->async_send_goal(left_goal);
    auto right_future = right_client_->async_send_goal(right_goal);

    // 步骤5：等待发送完成
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), left_future);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), right_future);

    RCLCPP_INFO(get_logger(), "轨迹已同步启动");

    // 步骤6：等待轨迹执行完成
    // int32_t sleeptime = left_goal.trajectory.points.back().time_from_start.sec;
    // RCLCPP_INFO(this->get_logger(), "sleeptime : %d",sleeptime);
    std::thread([this]()
                {
      std::this_thread::sleep_for(std::chrono::seconds(10));
      recording_active_ = false;
      save_data_to_csv();
      plot_joint_data(); })
        .detach();
  }

private:
  YAML::Node load_yaml_file()
  {
    std::filesystem::path config_path =
        std::filesystem::current_path() / "src/dual_arm_demo/config/dual_arm_config.yaml";
    std::string path = config_path.string();
    if (std::filesystem::exists(path))
    {
      RCLCPP_INFO(this->get_logger(), "ConfigFile path:%s", path.c_str());
    }
    else
    {
      throw std::runtime_error("Config file not found in: " + path);
    }

    return YAML::LoadFile(path);
  }

  bool wait_for_controllers(int timeout_sec)
  {
    if (!left_client_->wait_for_action_server(std::chrono::seconds(timeout_sec)) ||
        !right_client_->wait_for_action_server(std::chrono::seconds(timeout_sec)))
    {
      RCLCPP_ERROR(get_logger(), "控制器连接超时");
      return false;
    }
    return true;
  }

  void init_joint_data_containers()
  {
    for (const auto &joint : left_joints)
    {
      joint_data_["left_pos"][joint] = {};
      joint_data_["left_vel"][joint] = {};
    }

    for (const auto &joint : right_joints)
    {
      joint_data_["right_pos"][joint] = {};
      joint_data_["right_vel"][joint] = {};
    }
  }

  void clear_recorded_data()
  {
    time_points_.clear();
    for (auto &[data_type, joints] : joint_data_)
    {
      for (auto &[joint_name, values] : joints)
      {
        values.clear();
      }
    }
  }

  void joint_state_callback(std::shared_ptr<const sensor_msgs::msg::JointState> msg)
  {
    if (!recording_active_)
      return;
    static uint8_t cnt = 0;
    cnt++;
    if (cnt >= 255)
    {
      cnt = 0;
    }
    if (cnt % 50 == 1)
    {
      auto current_time = std::chrono::steady_clock::now();
      double elapsed_sec = std::chrono::duration<double>(current_time - start_time_).count();

      // 记录时间戳
      time_points_.push_back(elapsed_sec);

      // 记录关节数据
      for (size_t i = 0; i < msg->name.size(); ++i)
      {
        const auto &joint_name = msg->name[i];

        // 左臂数据处理
        if (joint_name.find("leftArm") != std::string::npos)
        {
          joint_data_["left_pos"][joint_name].push_back(msg->position[i]);
          joint_data_["left_vel"][joint_name].push_back(msg->velocity[i]);
        }
        // 右臂数据处理
        else if (joint_name.find("rightArm") != std::string::npos)
        {
          joint_data_["right_pos"][joint_name].push_back(msg->position[i]);
          joint_data_["right_vel"][joint_name].push_back(msg->velocity[i]);
        }
      }
    }
  }

  // 保存数据到CSV文件
  void save_data_to_csv()
  {
    std::ofstream file("joint_data.csv");

    // 写入表头
    file << "time";
    for (const auto &[joint_name, _] : joint_data_["left_pos"])
    {
      file << ",left_" << joint_name << "_pos,left_" << joint_name << "_vel";
    }
    for (const auto &[joint_name, _] : joint_data_["right_pos"])
    {
      file << ",right_" << joint_name << "_pos,right_" << joint_name << "_vel";
    }
    file << "\n";

    // 写入数据
    for (size_t i = 0; i < time_points_.size(); ++i)
    {
      file << time_points_[i];

      // 左臂数据
      for (const auto &[joint_name, values] : joint_data_["left_pos"])
      {
        file << "," << values[i] << "," << joint_data_["left_vel"][joint_name][i];
      }

      // 右臂数据
      for (const auto &[joint_name, values] : joint_data_["right_pos"])
      {
        file << "," << values[i] << "," << joint_data_["right_vel"][joint_name][i];
      }

      file << "\n";
    }

    RCLCPP_INFO(get_logger(), "数据已保存到 joint_data.csv");
  }

  // 绘制关节数据曲线
  void plot_joint_data()
  {
    using namespace matplot;

    // 创建2x2的子图布局
    auto figure = gcf();
    figure->size(1200, 800);

    // 1. 左臂位置曲线
    subplot(2, 2, 0);
    hold(on);
    for (const auto &[joint_name, values] : joint_data_["left_pos"])
    {
      plot(time_points_, values)->display_name(joint_name);
    }
    title("Left Arm Joint Positions");
    xlabel("Time (s)");
    ylabel("Position (rad)");
    legend();
    grid(on);

    // 2. 左臂速度曲线
    subplot(2, 2, 1);
    hold(on);
    for (const auto &[joint_name, values] : joint_data_["left_vel"])
    {
      plot(time_points_, values)->display_name(joint_name);
    }
    title("Left Arm Joint Velocities");
    xlabel("Time (s)");
    ylabel("Velocity (rad/s)");
    legend();
    grid(on);

    // 3. 右臂位置曲线
    subplot(2, 2, 2);
    hold(on);
    for (const auto &[joint_name, values] : joint_data_["right_pos"])
    {
      plot(time_points_, values)->display_name(joint_name);
    }
    title("Right Arm Joint Positions");
    xlabel("Time (s)");
    ylabel("Position (rad)");
    legend();
    grid(on);

    // 4. 右臂速度曲线
    subplot(2, 2, 3);
    hold(on);
    for (const auto &[joint_name, values] : joint_data_["right_vel"])
    {
      plot(time_points_, values)->display_name(joint_name);
    }
    title("Right Arm Joint Velocities");
    xlabel("Time (s)");
    ylabel("Velocity (rad/s)");
    legend();
    grid(on);

    // 保存图像
    save("joint_plots.png");
    RCLCPP_INFO(get_logger(), "曲线图已保存为 joint_plots.png");

    // 显示图像（需要GUI支持）
    show();
  }

  void Init_dual_arm_pose()
  {
    try
    {
      const double init_duartion = config["initial_positions"]["duration"].as<double>();
      FollowJointTrajectory::Goal left_init_goal;
      FollowJointTrajectory::Goal right_init_goal;
      trajectory_msgs::msg::JointTrajectoryPoint left_init_pose;
      trajectory_msgs::msg::JointTrajectoryPoint right_init_pose;

      left_init_pose.positions = config["initial_positions"]["left_arm"].as<std::vector<double>>();
      left_init_pose.time_from_start = rclcpp::Duration::from_seconds(init_duartion);
      left_init_goal.trajectory.points.push_back(left_init_pose);
      left_init_goal.trajectory.joint_names = left_joints;

      right_init_pose.positions = config["initial_positions"]["right_arm"].as<std::vector<double>>();
      right_init_pose.time_from_start = rclcpp::Duration::from_seconds(init_duartion);
      right_init_goal.trajectory.points.push_back(right_init_pose);
      right_init_goal.trajectory.joint_names = right_joints;

      auto now = this->now();
      left_init_goal.trajectory.header.stamp = now;
      right_init_goal.trajectory.header.stamp = now;

      auto left_init_future = this->left_client_->async_send_goal(left_init_goal);
      auto right_init_future = this->right_client_->async_send_goal(right_init_goal);

      rclcpp::spin_until_future_complete(this->get_node_base_interface(), left_init_future);
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), right_init_future);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception caught:%s", e.what());
    }

    RCLCPP_INFO(this->get_logger(), "正在运行至初始位置,5s后开始执行轨迹");
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }

  // 创建轨迹目标消息
  FollowJointTrajectory::Goal create_goal(const std::string &ns)
  {
    FollowJointTrajectory::Goal goal;
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> goal_points;

    // std::filesystem::path config_path =
    //     std::filesystem::current_path() / "src/dual_arm_demo/config/dual_arm_config.yaml";
    // std::string path = config_path.string();
    // if (std::filesystem::exists(path))
    // {
    //   RCLCPP_INFO(this->get_logger(), "ConfigFile path:%s", path.c_str());
    // }
    // else
    // {
    //   throw std::runtime_error("Config file not found in: " + path);
    // }

    // YAML::Node config = YAML::LoadFile(path);

    double time_interval = config["trajectory_params"]["time_interval"].as<double>();
    for (const auto &pos : config["trajectory_points"])
    {
      trajectory_msgs::msg::JointTrajectoryPoint point;

      int index = pos["index"].as<int>();
      if (ns == LEFT_NS)
      {
        point.positions = pos["left_arm"].as<std::vector<double>>();
        point.time_from_start = rclcpp::Duration::from_seconds(index * time_interval);
      }
      else
      {
        point.positions = pos["right_arm"].as<std::vector<double>>();
        point.time_from_start = rclcpp::Duration::from_seconds(index * time_interval);
      }

      goal_points.push_back(point);
    }

    // 关节名配置（根据实际URDF调整）
    if (ns == LEFT_NS)
    {
      goal.trajectory.joint_names = left_joints;
    }
    else
    {
      goal.trajectory.joint_names = right_joints;
    }
    // 添加多个轨迹点
    goal.trajectory.points = goal_points;
    return goal;
  }

  // Action客户端
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr left_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr right_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  std::map<std::string, std::map<std::string, std::vector<double>>> joint_data_;
  std::vector<double> time_points_;
  std::chrono::steady_clock::time_point start_time_;
  bool recording_active_ = false;
  YAML::Node config;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<DualArmExecutor>();
  executor->execute_trajectories();
  rclcpp::spin(executor);
  rclcpp::shutdown();
  return 0;
}