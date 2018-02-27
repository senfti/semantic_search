#!/bin/bash
rosbag record -e "/base_obj(.*)|/base_room(.*)|/obj_(.*)|/room_(.*)|/occupied_cells_vis_array|/mapper_door_poses|/gmap|/map|/map_door_blocked|/room_prob_map_view|/obj_prob_map_view|/base_obj_prob_map_view|/base_room_prob_map_view|/sdf|/hierarchy" -O /media/thomas/EE702FC8702F967D/studium/Masterarbeit/output/result_$1_$2.bag
