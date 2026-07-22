import rospy
import os
import rosbag
from nav_msgs.msg import Path
import tf
import math
import numpy as np
import cv2

from nav_msgs.msg import Path

pub = rospy.Publisher('/pathnew', Path, queue_size=1)

def callback(data):
    start = data.poses[-1].header.stamp.to_sec() - 300
    out = Path()
    out.header = data.header
    for p in data.poses:
        if p.header.stamp.to_sec() > start:
            out.poses.append(p)

    pub.publish(out)

rospy.init_node('path_repub', anonymous=True)
rospy.Subscriber("/robot_path", Path, callback, queue_size=1)
rospy.spin()