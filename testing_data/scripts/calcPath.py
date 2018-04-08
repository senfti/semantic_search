import rospy
import os
import rosbag
from nav_msgs.msg import Path
import tf
import math

files = ["wzs3_fSpoonUnexp.bag", "wzs4_fSpoonUnexp.bag", "wzs5_fSpoonUnexp.bag", "wzs6_sSpoonUnexp.bag", "wzs8_sSpoonUnexp.bag", "wzs9_sSpoonUnexp.bag", "wzs10_sSpoonUnexp.bag",
         "wzs11_sSpoonUnexp.bag", "wzs12_fSpoonUnexp.bag", "wzs13_fSpoonUnexp.bag", "wzs14_RemoteUnexp.bag"]

for filename in files:
    folder = "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/"
    bag = rosbag.Bag(folder+filename)

    path_msgs = bag.read_messages(topics="/robot_path")
    start_t = 0
    for topic, msg, t in path_msgs:
        if start_t==0:
            start_t = t
        path = msg

    action_msgs = bag.read_messages(topics="/current_action")
    old_data = -1
    for topic, msg, t in action_msgs:
        print msg.data
        if old_data==1 and msg.data!=1:
            explore_end = t
            break
        old_data = msg.data

    print t.to_sec()-start_t.to_sec(), explore_end.to_sec()-start_t.to_sec()

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

    print filename
    print dist, ang_dist
    print explore_dist, explore_ang_dist
    print
