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

bag = rosbag.Bag("/home/thomas/Desktop/h3e.bag")
found_msgs = bag.read_messages(topics="/camera/rgb/image_raw")
skip = 0
for topic, msg, t in found_msgs:
    if skip > 0:
        skip = skip-1
        continue
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Image window", cv_image)
    if cv2.waitKey(0) == ord('s'):
        skip = 10