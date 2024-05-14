# RP-Robotic-Arm
## How To Run
1. Edit **file_subpath** in RP_Arm.launch.py with xacro file path
2. Build the package with `colcon build`
3. Source and setup the environment `source ~/.bashrc` then `source install/setup.bash`
4. Launch `robot_state_publisher` launch file with `ros2 launch rp_arm_bringup RP_Arm.launch.py`
5. Launch RViz with `rviz2`
	- Set your **Fixed Frame** to `world`
	- **Add** a `RobotModel` display, with the **Description topic** set to `/robot_description` 
6. Launch the script with `ros2  run  pkg_rp_arm exec_rp_arm`
7. Launch `teleop_twist_keyboard` with `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
