import rospy
import os
import rosbag
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray
import tf
import math

rospy.init_node('doorway_vis', anonymous=True)
dpub = rospy.Publisher('/doorways', PoseArray, queue_size=1)
mpub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)

filenames = ["/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/remote_table_s11.bag"]
num_doorways = 0
num_files = 0
for filename in filenames:
    bag = rosbag.Bag(filename)
    print filename
    action_msgs = bag.read_messages(topics=['/mapper_door_poses'])
    for topic, msg, t in action_msgs:
        poses = msg
    


filename = "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/remote_table_s11.bag"
bag = rosbag.Bag(filename)
print filename

action_msgs = bag.read_messages(topics="/current_action")
old = -1
for topic, msg, t in action_msgs:
    if (old==1) and msg.data!=1:
        exp_time = t
        print "sdf", t.to_sec()
    if old==2 and (msg.data!=2):
        search_time = t
        print "sdf3", t.to_sec()
    if msg.data != old:
        print msg.data
    old = msg.data

map_msg = bag.read_messages(topics="/map", start_time=exp_time)
for topic, msg, t in map_msg:
    mm=msg
    mpub.publish(msg)
    break
ep_msg = bag.read_messages(topics="/robot_path", start_time=exp_time)
for topic, msg, t in ep_msg:
    ep = msg
    break
ep_msg = bag.read_messages(topics="/robot_path", start_time=search_time)
for topic, msg, t in ep_msg:
    fp = msg
    plist = list(msg.poses)
    idx = 0
    idx2 = 1
    for e in plist:
        if e.header.stamp.to_sec() > exp_time.to_sec():
            idx += 1
        if e.header.stamp.to_sec() < search_time.to_sec():
            idx2 += 1
    plist = plist[idx:idx2]
    msg.poses = tuple(plist)
    sp = msg
    break

while True:
    mpub.publish(mm)
    epub.publish(ep)
    spub.publish(sp)
    fpub.publish(fp)
    rospy.sleep(0.5)