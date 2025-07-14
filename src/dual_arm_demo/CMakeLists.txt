cmake_minimum_required(VERSION 3.8)
project(dual_arm_demo)

# 编译器设置
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 基础依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(sensor_msgs REQUIRED)

# 查找YAML-CPP
find_package(yaml-cpp 0.7.0 REQUIRED)
ament_export_dependencies(yaml-cpp)

find_package(Matplot++ REQUIRED)

# 添加可执行文件
add_executable(dual_arm_demo src/dual_arm_node.cpp)
add_executable(dual_arm_move src/dual_arm_move.cpp)

# 为dual_arm_move设置依赖项 - 全部使用无关键字签名
target_include_directories(dual_arm_move PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${control_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${Matplot++_INCLUDE_DIRS}
)

# 只使用一次target_link_libraries - 无关键字签名
target_link_libraries(dual_arm_move
  ${rclcpp_LIBRARIES}
  ${rclcpp_action_LIBRARIES}
  ${control_msgs_LIBRARIES}
  ${trajectory_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  Matplot++::matplot
  yaml-cpp
)


target_compile_definitions(dual_arm_move PRIVATE 
  YAML_CPP_STATIC_DEFINE  # 解决符号未定义问题[2,6](@ref)
)

# 为dual_arm_demo设置依赖项
ament_target_dependencies(
  dual_arm_demo
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
)

# 安装配置
install(TARGETS 
  dual_arm_demo
  dual_arm_move
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  include/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME})

# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()