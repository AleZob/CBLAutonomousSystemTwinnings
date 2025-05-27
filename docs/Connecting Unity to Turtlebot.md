# Setting up unity
Open Unity project
Open workspace in terminal
1. source /opt/ros/foxy/setup.bash
2. source install/local_setup.bash 
Hit play in Unity

# Connecting turtlebot3 to Unity
1. ssh ubuntu@{IP robot} (can be found on the robot)
2. export TURTLEBOT3_MODEL=burger
3. ros2 launch turtlebot3_bringup robot.launch.py

## Open a new terminal 
4. source install/setup.bash 
5. ros2 run ros_tcp_endpoint  default_server_endpoint --ros-args -p ROS_IP:= <laptop IP> (check on the back of the laptop)
6. Hit play on Unity

***Verify with blue arrows that the connection is active***

Now we can launch any node that the physical and virtual robot will subscribe to at the 
same time!

# Moving the turtlebot3 
Open a new terminal
1. export TURTLEBOT3_MODEL=burger
2. ros2 run turtlebot3_teleop teleop_keyboard

# Using RViz to Visualize Surroundings in Digital Environment
Open a new terminal in the source directory of workspace
1. source /opt/ros/foxy/setup.bash
2. source install/local_setup.bash
3. export TURTLEBOT3_MODEL=waffle
4. ros2 launch unity_slam_example unity_slam_example.py

Next proceed by using teleop keyboard to move the robot. 
