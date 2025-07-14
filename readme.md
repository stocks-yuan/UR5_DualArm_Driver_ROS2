# UR5_DualArm_Driver
## Quick Start
```bash
bash start_sim_dual_ur5.sh
```
此指令分为三步：
- 第一步：调用`bash ./src/ursim_script/docker_dual_ur5.sh`，启动双臂UR_SIM仿真
- 第二步：调用 `ros2 launch dual_ur5_arm_driver start_dual_arm_driver.launch.py`命令，启动双臂驱动程序
- 第三步：调用`ros2 launch dual_arm_moveit_config dual_moveit.launch.py`命令，启动Moveit功能包

> 若不想调用`bash start_sim_dual_ur5.sh`，可以按照顺序分别调用

## Project architecture

```bash
src	#程序源码
├── dual_arm_demo	#双臂demo程序
├── dual_arm_moveit_config	#Moveit2程序
├── dual_description	#双臂硬件描述文件
├── dual_ur5_arm_driver	#双臂驱动文件，基于Universal_Robots_ROS2_Driver
├── libs	#库文件
├── Universal_Robots_ROS2_Driver	#UR机械臂官方驱动（单臂）
└── ursim_script	#双臂UR_SIM仿真文件
doc
├── dual_ur5_ft_gripper_base.gv
├── dual_ur5_ft_gripper_base.pdf	#URDF文件输出的pdf
└── output.urdf	#双臂URDF架构
```

