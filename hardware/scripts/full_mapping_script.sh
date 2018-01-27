#!/bin/bash
roslaunch hardware 1_turtlebot.launch &
rosclean purge -y &
sleep 2
roslaunch hardware 2_cam1_laser.launch &
sleep 2
roslaunch hardware 3_cam2_laser_filter.launch &
sleep 4
roslaunch hardware 4_cam1_settings_move_base.launch &
sleep 4
roslaunch hardware 5_cam2_settings_footprint.launch &
sleep 6
xterm -hold -e roslaunch hardware 6_door_detection.launch &
sleep 4
xterm -hold -e roslaunch hardware 7_vision.launch &
sleep 4
xterm -hold -e roslaunch hardware 8_mapping.launch &
sleep 1
xterm -hold -e roslaunch hardware 9_execution.launch &
sleep 1
xterm -hold -e roslaunch hardware 10_hl_planner.launch &

