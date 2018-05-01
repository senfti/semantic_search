import rospy
import os
import rosbag
from nav_msgs.msg import Path
import tf
import math
import numpy as np
import cv2

import visualization_msgs.msg._MarkerArray
import std_msgs.msg._ColorRGBA

pub = rospy.Publisher('/occ_map3d', visualization_msgs.msg.MarkerArray, queue_size=1)

def callback(data):
    for m in range(len(data.markers)):
        remove_list = []
        for i in range(len(data.markers[m].colors)):
            data.markers[m].colors[i].r = 1
            data.markers[m].colors[i].g = 0
            data.markers[m].colors[i].b = 0
            if(data.markers[m].points[i].z < 0.9 or data.markers[m].points[i].x + data.markers[m].points[i].y < 1 ):
                data.markers[m].colors[i].a = 1
            else:
                remove_list.insert(0,i)
        for i in remove_list:
            data.markers[m].colors.pop(i)
            data.markers[m].points.pop(i)

    pub.publish(data)

rospy.init_node('occ_repub', anonymous=True)
rospy.Subscriber("/occupied_cells_vis_array", visualization_msgs.msg.MarkerArray, callback, queue_size=1)
rospy.spin()