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
#"/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/full35_Spoon_Table_full_unexplored.bag",
"/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/full37_Spoon_Table_stupid_search_unexplored.bag"]

move_to_times = []
explore_times = []
search_times = []
turn_times = []
full_times = []
move_to_dists = []
explore_dists = []
search_dists = []
turn_dists = []
full_dists = []
move_to_ang_dists = []
explore_ang_dists = []
search_ang_dists = []
turn_ang_dists = []
full_ang_dists = []

for filename in files:
    folder = "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/"
    bag = rosbag.Bag(filename)
    print filename

    found_msgs = bag.read_messages(topics="/object_found_pose")
    for topic, msg, t in found_msgs:
        print "found"

    path_msgs = bag.read_messages(topics="/robot_path")
    for topic, msg, t in path_msgs:
        path = msg

    dist_array = []
    ang_dist_array = []
    array_times = []
    dist=0.0
    ang_dist=0.0
    for i in range(1,len(path.poses)):
        x = path.poses[i].pose.position.x-path.poses[i-1].pose.position.x
        y = path.poses[i].pose.position.y-path.poses[i-1].pose.position.y
        dist += math.sqrt(x*x+y*y)
        quaternion = (path.poses[i].pose.orientation.x,path.poses[i].pose.orientation.y,path.poses[i].pose.orientation.z,path.poses[i].pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        ang_dist += math.fabs(euler[2])

        dist_array.append(dist)
        ang_dist_array.append(ang_dist)
        array_times.append(path.poses[i].header.stamp.to_sec())

    action_msgs = bag.read_messages(topics="/current_action")
    old_data = -1
    last_segment_start_time = 0
    start = 0

    move_to_time = 0
    explore_time = 0
    search_time = 0
    turn_time = 0
    full_time = 0
    move_to_dist = 0
    explore_dist = 0
    search_dist = 0
    turn_dist = 0
    full_dist = 0
    move_to_ang_dist = 0
    explore_ang_dist = 0
    search_ang_dist = 0
    turn_ang_dist = 0
    full_ang_dist = 0

    for topic, msg, t in action_msgs:
        if old_data==-1 and msg.data!=-1:
            last_segment_start_time = t.to_sec()
        if start==0 and (msg.data!=-1):
            start = t.to_sec()

        if msg.data==-1 and old_data==0:
            move_to_time+=t.to_sec()-last_segment_start_time
            idx = next(x[0] for x in enumerate(array_times) if x[1] > t.to_sec())
            start_idx = next(x[0] for x in enumerate(array_times) if x[1] > last_segment_start_time)
            move_to_dist += dist_array[idx]-dist_array[start_idx]
            move_to_ang_dist += ang_dist_array[idx]-ang_dist_array[start_idx]
            full_time=t.to_sec()-start
            full_dist=dist_array[idx]
            full_ang_dist=ang_dist_array[idx]

        if msg.data==-1 and old_data==1:
            explore_time+=t.to_sec()-last_segment_start_time
            idx = next(x[0] for x in enumerate(array_times) if x[1] > t.to_sec())
            start_idx = next(x[0] for x in enumerate(array_times) if x[1] > last_segment_start_time)
            explore_dist += dist_array[idx]-dist_array[start_idx]
            explore_ang_dist += ang_dist_array[idx]-ang_dist_array[start_idx]
            full_time=t.to_sec()-start
            full_dist=dist_array[idx]
            full_ang_dist=ang_dist_array[idx]

        if msg.data==-1 and old_data==2:
            search_time+=t.to_sec()-last_segment_start_time
            idx = next(x[0] for x in enumerate(array_times) if x[1] > t.to_sec())
            start_idx = next(x[0] for x in enumerate(array_times) if x[1] > last_segment_start_time)
            search_dist += dist_array[idx]-dist_array[start_idx]
            search_ang_dist += ang_dist_array[idx]-ang_dist_array[start_idx]
            full_time=t.to_sec()-start
            full_dist=dist_array[idx]
            full_ang_dist=ang_dist_array[idx]

        if msg.data==-1 and (old_data==4 or old_data==5):
            turn_time+=t.to_sec()-last_segment_start_time
            idx = next(x[0] for x in enumerate(array_times) if x[1] > t.to_sec())
            start_idx = next(x[0] for x in enumerate(array_times) if x[1] > last_segment_start_time)
            turn_dist += dist_array[idx]-dist_array[start_idx]
            turn_ang_dist += ang_dist_array[idx]-ang_dist_array[start_idx]
            full_time=t.to_sec()-start
            full_dist=dist_array[idx]
            full_ang_dist=ang_dist_array[idx]


        old_data = msg.data

    move_to_times.append(move_to_time)
    explore_times.append(explore_time)
    search_times.append(search_time)
    turn_times.append(turn_time)
    full_times.append(full_time)
    move_to_dists.append(move_to_dist)
    explore_dists.append(explore_dist)
    search_dists.append(search_dist)
    turn_dists.append(turn_dist)
    full_dists.append(full_dist)
    move_to_ang_dists.append(move_to_ang_dist)
    explore_ang_dists.append(explore_ang_dist)
    search_ang_dists.append(search_ang_dist)
    turn_ang_dists.append(turn_ang_dist)
    full_ang_dists.append(full_ang_dist)


for i in range(len(full_times)):
    print full_times[i], move_to_times[i], explore_times[i], search_times[i], turn_times[i]
print

for i in range(len(full_dists)):
    print full_dists[i], move_to_dists[i], explore_dists[i], search_dists[i], turn_dists[i]
print

for i in range(len(search_dists)):
    print full_ang_dists[i], move_to_ang_dists[i], explore_ang_dists[i], search_ang_dists[i], turn_ang_dists[i]
print