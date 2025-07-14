#!/bin/bash

# Enable error handling
set -e
set -o pipefail
set -u

LOG_FILE="dual_ur5_setup.log"
exec > >(tee -i "$LOG_FILE") 2>&1

echo "Starting Dual UR5 simulation setup script with multiple terminals..."
echo "==================================================================="

# Step 1: Run docker_dual_ur5.sh in a new terminal
echo "Opening a new terminal for docker_dual_ur5.sh..."
gnome-terminal -- bash -c "./src/ursim_script/docker_dual_ur5.sh; exec bash"

# Wait for user input before proceeding to Step 2
echo "Step 1 has finished. Press any key to continue to Step 2..."
read -n 1 -s  # Wait for the user to press any key

# Step 2: Run ROS2 dual arm driver in a new terminal
echo "Opening a new terminal for ROS2 dual arm driver..."
gnome-terminal -- bash -c "source install/local_setup.sh; ros2 launch dual_ur5_arm_driver start_dual_arm_driver.launch.py; exec bash"

# Wait for user input before proceeding to Step 3
echo "Step 2 has finished. Press any key to continue to Step 3..."
read -n 1 -s  # Wait for the user to press any key


# Step 3: Run MoveIt configuration in a new terminal
echo "Opening a new terminal for MoveIt configuration..."
gnome-terminal -- bash -c "source install/local_setup.sh; ros2 launch dual_arm_moveit_config dual_moveit.launch.py; exec bash"

echo "All processes started in separate terminals. Check logs in each terminal."

