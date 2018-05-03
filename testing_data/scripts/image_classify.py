import rospy
import os
import rosbag
from nav_msgs.msg import Path
import tf
import math
import numpy as np
import cv2
import time

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('sdf', anonymous=True)
bridge = CvBridge()
pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=1)
pub_cloud = rospy.Publisher('/camera/depth_registered/points', PointCloud2, queue_size=1)

bag = rosbag.Bag("/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/explore_lab_1.bag")
found_msgs = bag.read_messages(topics="/camera/depth_registered/points")
for topic, msg, t in found_msgs:
    pub_cloud.publish(msg)
    break

found_msgs = bag.read_messages(topics="/camera/rgb/image_raw")
for topic, msg, t in found_msgs:
    pub.publish(msg)
    time.sleep(0.2)