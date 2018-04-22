#!/bin/bash
roslaunch hardware 1_turtlebot.launch &
sleep 2
rosclean purge -y &
sleep 2
if [ $# -gt 1 ]
  then
    xterm -hold -e roslaunch hardware 2_cam1_laser.launch cam_nr:="#2" &
  else
    xterm -hold -e roslaunch hardware 2_cam1_laser.launch &
fi
sleep 2
if [ $# -gt 1 ]
  then
    xterm -hold -e roslaunch hardware 3_cam2_laser_filter.launch cam_nr:="#1" &
  else
    xterm -hold -e roslaunch hardware 3_cam2_laser_filter.launch &
fi
sleep 3
xterm -hold -e roslaunch hardware 4_cam1_settings_move_base.launch &
sleep 3
xterm -hold -e roslaunch hardware 5_cam2_settings_footprint.launch &
sleep 3
#xterm -hold -e roslaunch hardware 6_door_detection.launch &
sleep 2
xterm -hold -e roslaunch hardware 7_vision.launch &
sleep 4
xterm -hold -e roslaunch hardware 8_mapping.launch &
sleep 1
xterm -e rviz &
xterm -hold -e roslaunch hardware 9_execution.launch &
sleep 5
xterm -hold -e roslaunch hardware 10_hl_planner.launch RUN_NAME:="$1" &
sleep 5
if [ $# -gt 0 ]
  then
    source /home/thomas/Desktop/rosbag_record5.sh $1
fi

