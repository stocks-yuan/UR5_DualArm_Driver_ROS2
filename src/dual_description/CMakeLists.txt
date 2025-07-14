cmake_minimum_required(VERSION 3.8)
project(dual_description)

# 如果编译器是 GNUC++ 或 Clang，添加编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置依赖项
set(dependencies
  ament_cmake
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2
  rclcpp_action
  rclcpp_components
  control_msgs
  moveit_common
  moveit_visual_tools
  moveit_ros_planning
)

# 查找依赖项
foreach(dependency ${dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

# 创建库
add_library(dual_ur_gripper_node src/dual_ur_gripper_node.cpp)
# 要求 C99 和 C++17 标准
target_compile_features(dual_ur_gripper_node PUBLIC c_std_99 cxx_std_17)
# 设置包含目录
target_include_directories(dual_ur_gripper_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# 设置库的依赖项
ament_target_dependencies(dual_ur_gripper_node ${dependencies})

# 设置编译定义
target_compile_definitions(dual_ur_gripper_node PRIVATE "TEST_CREATING_BUILDING_LIBRARY")

# 安装目标和目录
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS dual_ur_gripper_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# # 创建可执行文件
# add_executable(dual_ur5_robotiq_description src/dual_ur5_robotiq_description.cpp)
# # 设置包含目录
# target_include_directories(dual_ur5_robotiq_description PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# # 要求 C99 和 C++17 标准
# target_compile_features(dual_ur5_robotiq_description PUBLIC c_std_99 cxx_std_17)
# # 设置可执行文件的依赖项
# ament_target_dependencies(dual_ur5_robotiq_description ${dependencies})

# # 安装可执行文件和其他目录
# install(TARGETS dual_ur5_robotiq_description
#   DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY
    config
    launch
    rviz
    urdf
    gazebo
    srdf
    moveit2
    meshes
  DESTINATION
    share/${PROJECT_NAME}
)

# 构建测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 设置 cpplint 跳过检查
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 导出包含目录和依赖项
ament_export_include_directories(include)
ament_export_libraries(dual_ur_gripper_node)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
