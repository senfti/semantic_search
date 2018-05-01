import rospy
import os
import rosbag
from nav_msgs.msg import Path
import tf
import math
import numpy as np
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

bag = rosbag.Bag("/home/thomas/Desktop/IST_expolere_su.bag")
found_msgs = bag.read_messages(topics="/camera/rgb/image_raw")
for topic, msg, t in found_msgs:
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(0)