#!/bin/sh
export TURTLEBOT_GAZEBO_MAP_FILE='/home/robond/workspace/RoboND-Final/catkin_ws/src/map/map.yaml'
xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 2
xterm -e " source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/robond/workspace/RoboND-Final/catkin_ws/src/map/myworld.world " &
sleep 2
xterm -e " source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 2
xterm -e " source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm -e " source devel/setup.bash; rosrun add_markers add_markers _mode:=test "