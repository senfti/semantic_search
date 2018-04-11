import rospy
import os
import rosbag
from nav_msgs.msg import Path
import tf
import math

files = ["wzs35_Remote_Table_stupid_unexplored.bag","wzs37_Remote_Table_full_unexplored.bag","wzs38_Remote_Table_stupid_unexplored.bag","wzs39_Remote_Table_full_unexplored.bag"]

explore_times = []
search_times = []
search_dists = []
explore_dists = []
search_ang_dists = []
explore_ang_dists = []

for filename in files:
    folder = "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/"
    bag = rosbag.Bag(folder+filename)
    print folder+filename

    path_msgs = bag.read_messages(topics="/robot_path")
    for topic, msg, t in path_msgs:
        path = msg

    action_msgs = bag.read_messages(topics="/current_action")
    old_data = -1
    start = 0.0
    explore_end = 0.0
    search_end = 0.0
    for topic, msg, t in action_msgs:
        if start==0 and (msg.data==5 or msg.data==1):
            start = t
        if explore_end==0 and old_data==1 and msg.data!=1:
            explore_end = t
        if search_end==0 and old_data==2 and msg.data!=2:
            search_end = t

        old_data = msg.data

    explore_times.append(explore_end.to_sec()-start.to_sec())
    search_times.append(search_end.to_sec()-start.to_sec())

    explore_dist = 0.0
    explore_ang_dist = 0.0
    dist = 0.0
    ang_dist = 0.0
    for i in range(1,len(path.poses)):
        x = path.poses[i].pose.position.x-path.poses[i-1].pose.position.x
        y = path.poses[i].pose.position.y-path.poses[i-1].pose.position.y
        dist += math.sqrt(x*x+y*y)
        quaternion = (path.poses[i].pose.orientation.x,path.poses[i].pose.orientation.y,path.poses[i].pose.orientation.z,path.poses[i].pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        ang_dist += math.fabs(euler[2])
        if math.fabs(euler[2]) > 3.14:
            print "O SHIT O SHIT O SHIT"

        if path.poses[i].header.stamp < explore_end:
            explore_dist += math.sqrt(x * x + y * y)
            explore_ang_dist += math.fabs(euler[2])

    search_dists.append(dist)
    search_ang_dists.append(ang_dist)
    explore_dists.append(explore_dist)
    explore_ang_dists.append(explore_ang_dist)


for i in range(len(search_dists)):
    print search_times[i], explore_times[i]
print

for i in range(len(search_dists)):
    print search_dists[i], explore_dists[i]
print

for i in range(len(search_dists)):
    print search_ang_dists[i], explore_ang_dists[i]
print