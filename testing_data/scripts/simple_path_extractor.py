import rospy
import os
import rosbag
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
import tf
import math

rospy.init_node('path_split', anonymous=True)
epub = rospy.Publisher('/pe', Path, queue_size=1, latch=True)
spub = rospy.Publisher('/ps', Path, queue_size=1, latch=True)
fpub = rospy.Publisher('/pf', Path, queue_size=1, latch=True)
mpub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)

while True:
    filename = raw_input("file: ")
    if len(filename) < 2:
        break
    filename = "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/" + filename + ".bag"
    bag = rosbag.Bag(filename)
    print filename

    map_msg = bag.read_messages(topics="/map")
    for topic, msg, t in map_msg:
        mm=msg

    map_msg = bag.read_messages(topics="/robot_path")
    for topic, msg, t in map_msg:
        p=msg
    spub.publish(p,)
    mpub.publish(mm)