This package is the lab 0 for ROB 456.
It was ported to ROS2 on 11/7/23 by Leopold Klotz.

1. create a ros2 workspace: mkdir -p ~/ros2_ws/src && cd ~/ros2_ws && colcon build
2. copy the labzero package into the src folder
3. build the package: cd ~/ros2_ws && colcon build --packages-select labzero
4. source the package: source ~/ros2_ws/install/setup.bash
5. run the package: ros2 launch labzero circle.launch.py