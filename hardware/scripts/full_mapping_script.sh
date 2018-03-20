#!/bin/bash
roslaunch hardware 1_turtlebot.launch &
sleep 3
rosclean purge -y &
sleep 2
if [ $# -eq 0 ]
  then
    xterm -hold -e roslaunch hardware 2_cam1_laser.launch &
  else
    xterm -hold -e roslaunch hardware 2_cam1_laser.launch cam_nr:="#2" &
fi
sleep 5
if [ $# -eq 0 ]
  then
    xterm -hold -e roslaunch hardware 3_cam2_laser_filter.launch &
  else
    xterm -hold -e roslaunch hardware 3_cam2_laser_filter.launch cam_nr:="#1" &
fi
sleep 6
xterm -hold -e roslaunch hardware 4_cam1_settings_move_base.launch &
sleep 6
xterm -hold -e roslaunch hardware 5_cam2_settings_footprint.launch &
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

