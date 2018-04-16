import rospy
import os
import rosbag
from nav_msgs.msg import Path
import tf
import math

files = ["/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/full21_Remote_wzTable_full_unexplored.bag",
"/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/full22_Remote_wzTable_full_unexplored.bag",
"/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/full25_Spoon_Table_full_unexplored.bag",
"/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/full26_Spoon_Table_full_unexplored.bag",
"/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/full27_Remote_wzTable_stupid_search_unexplored.bag",
"/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/full28_Remote_wzTable_stupid_search_unexplored.bag",
"/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/full29_Remote_wzTable_stupid_search_unexplored.bag",
"/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/full30_Remote_wzTable_stupid_explore_unexplored.bag",
"/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/full31_Remote_wzTable_stupid_explore_unexplored.bag",
"/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/full33_Remote_wzTable_stupid_explore_unexplored.bag",
"/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/full34_Spoon_Table_stupid_explore_unexplored.bag",
"/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/full37_Spoon_Table_stupid_search_unexplored.bag"]

for filename in files:
    folder = "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/"
    bag = rosbag.Bag(filename)
    print filename

    path_msgs = bag.read_messages(topics="/robot_path")
    for topic, msg, t in path_msgs:
        path = msg

    exp_starts = []
    exp_ends = []

    action_msgs = bag.read_messages(topics="/current_action")
    old = 2
    for topic, msg, t in action_msgs:
        if (old!=2) and msg.data==2:
            exp_ends.append(t.to_sec())
        if old==2 and (msg.data!=2):
            exp_starts.append(t.to_sec())
        old = msg.data

    print filename+'explore_path.bag'
    out = rosbag.Bag(filename+'explore_path.bag', 'w')
    for i in range(len(exp_ends)):
        exp_path = Path()
        exp_path.header = path.header
        for j in range(1, len(path.poses)):
            if path.poses[j].header.stamp.to_sec() > exp_starts[i] and path.poses[j].header.stamp.to_sec() < exp_ends[i]:
                exp_path.poses.append(path.poses[j])
        out.write('exp_path' + str(i), exp_path)

    out.close()
