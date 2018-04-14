import subprocess
import signal
import time
import rospy
import os
from rosgraph_msgs.msg import Clock
from subprocess import check_output

def createDat(param):
    assoc = divisi2.network.conceptnet_assoc('en')
    U, S, _ = assoc.svd(k=100)
    spread = divisi2.reconstruct_activation(U, S)

    with open("/home/thomas/BVLCcaffe/models/googlenet_places205/categories_places205.csv") as f:
        content = f.readlines()
    f.close()
    content = [x.strip() for x in content]
    places = []
    for c in content:
        if c[0] != '_':
            places.append(c)

    f = open('/home/thomas/semantic_search_git/semantic_search/semantic_mapping_v2/data/room_spread.dat', 'w')
    i = 1
    #f.write(' ')
    #for w2 in content:
    #    f.write(w2 + ' ')
    #f.write('\n')

    for i1,w1 in enumerate(places):
    #    f.write(w1 + ' ')
        ss = []
        sum = 0.0
        #min = 0.0
        for i2,w2 in enumerate(places):
            try:
                s = max(spread.entry_named(w1, w2), 0.01)
                if i1 == i2:
                    s = param
                else:
                    sum += s
                ss.append(s)
                if(s < 0):
                    print "lsdkfjsdlfkj%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
            except KeyError:
                print i, w1, w2
                i+=1
        print

        for i2,s in enumerate(ss):
            if i1 != i2:
                s = s / sum * (1.0-param)
            f.write(str(s) + ' ')
        f.write('\n')

param = [0.05, 0.1, 0.15, 0.2, 0.25, 0.27, 0.3, 0.35, 0.4, 0.45, 0.5, 0.6]

def terminate_process_and_children(p):
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    assert retcode == 0, "ps command returned %d" % retcode
    for pid_str in ps_output.split("\n")[:-1]:
            os.kill(int(pid_str), signal.SIGINT)
    p.terminate()

def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            os.system("rosnode kill " + str)

last_clock = rospy.Time(0)
def callback(data):
    global last_clock
    last_clock = data


subprocess.Popen('xterm -e roscore', shell=True)
time.sleep(5)
rospy.init_node('mapping_test_script', anonymous=True)
rospy.Subscriber("/clock", Clock, callback, queue_size=1)
pub = rospy.Publisher('/clock', Clock, queue_size=1)

for p in param:
    createDat(p)

    print p
    proc_map = subprocess.Popen('xterm -e roslaunch hardware semantic_mapping_v2_rosbag.launch', shell=True)
    time.sleep(5)

    proc_rosbag = subprocess.Popen('xterm -e rosbag play --clock /media/thomas/EE702FC8702F967D/studium/Masterarbeit/data/robolab_2.bag', shell=True)
    proc_rosbag.wait()

    proc_hierarchy = subprocess.Popen('xterm -e rosservice call /hierarchy_srv "debug_room: 0"', shell=True)
    proc_hierarchy.wait()

    print 'xterm -e rosbag record -e "/base_obj(.*)|/base_room(.*)|/obj_(.*)|/room_(.*)|/occupied_cells_vis_array|/mapper_door_poses|/gmap|/map|/map_door_blocked|/room_prob_map_view|/obj_prob_map_view|/base_obj_prob_map_view|/base_room_prob_map_view|/sdf|/hierarchy" -O /media/thomas/EE702FC8702F967D/studium/Masterarbeit/output/result_test_rl_' + str(p) +'.bag'
    proc_save = subprocess.Popen('xterm -e rosbag record -e "/base_obj(.*)|/base_room(.*)|/obj_(.*)|/room_(.*)|/occupied_cells_vis_array|/mapper_door_poses|/gmap|/map|/map_door_blocked|/room_prob_map_view|/obj_prob_map_view|/base_obj_prob_map_view|/base_room_prob_map_view|/sdf|/hierarchy" -O /media/thomas/EE702FC8702F967D/studium/Masterarbeit/output/result_test_rl_' + str(p) +'.bag', shell=True)
    time.sleep(3)
    last_clock.clock = last_clock.clock + rospy.Duration(0.1)
    pub.publish(last_clock)
    time.sleep(3)

    proc_hierarchy = subprocess.Popen('xterm -e rosservice call /hierarchy_srv "debug_room: 0"', shell=True)
    proc_hierarchy.wait()
    time.sleep(5)
    terminate_ros_node("/record")

    time.sleep(3)
    terminate_process_and_children(proc_map)
    time.sleep(3)

for p in param:
    createDat(p)

    print p
    proc_map = subprocess.Popen('xterm -e roslaunch hardware semantic_mapping_v2_rosbag.launch', shell=True)
    time.sleep(5)

    proc_rosbag = subprocess.Popen('xterm -e rosbag play --clock /media/thomas/EE702FC8702F967D/studium/Masterarbeit/data/home_asus_0.bag', shell=True)
    proc_rosbag.wait()

    proc_hierarchy = subprocess.Popen('xterm -e rosservice call /hierarchy_srv "debug_room: 0"', shell=True)
    proc_hierarchy.wait()

    print 'xterm -e rosbag record -e "/base_obj(.*)|/base_room(.*)|/obj_(.*)|/room_(.*)|/occupied_cells_vis_array|/mapper_door_poses|/gmap|/map|/map_door_blocked|/room_prob_map_view|/obj_prob_map_view|/base_obj_prob_map_view|/base_room_prob_map_view|/sdf|/hierarchy" -O /media/thomas/EE702FC8702F967D/studium/Masterarbeit/output/result_test_ws_' + str(p) +'.bag'
    proc_save = subprocess.Popen('xterm -e rosbag record -e "/base_obj(.*)|/base_room(.*)|/obj_(.*)|/room_(.*)|/occupied_cells_vis_array|/mapper_door_poses|/gmap|/map|/map_door_blocked|/room_prob_map_view|/obj_prob_map_view|/base_obj_prob_map_view|/base_room_prob_map_view|/sdf|/hierarchy" -O /media/thomas/EE702FC8702F967D/studium/Masterarbeit/output/result_test_ws_' + str(p) +'.bag', shell=True)
    time.sleep(3)
    last_clock.clock = last_clock.clock + rospy.Duration(0.1)
    pub.publish(last_clock)
    time.sleep(3)

    proc_hierarchy = subprocess.Popen('xterm -e rosservice call /hierarchy_srv "debug_room: 0"', shell=True)
    proc_hierarchy.wait()
    time.sleep(5)
    terminate_ros_node("/record")

    time.sleep(3)
    terminate_process_and_children(proc_map)
    time.sleep(3)

for p in param:
    createDat(p)

    print p
    proc_map = subprocess.Popen('xterm -e roslaunch hardware semantic_mapping_v2_rosbag.launch', shell=True)
    time.sleep(5)

    proc_rosbag = subprocess.Popen('xterm -e rosbag play --clock /media/thomas/EE702FC8702F967D/studium/Masterarbeit/data/home_asus_2.bag', shell=True)
    proc_rosbag.wait()

    proc_hierarchy = subprocess.Popen('xterm -e rosservice call /hierarchy_srv "debug_room: 0"', shell=True)
    proc_hierarchy.wait()

    print 'xterm -e rosbag record -e "/base_obj(.*)|/base_room(.*)|/obj_(.*)|/room_(.*)|/occupied_cells_vis_array|/mapper_door_poses|/gmap|/map|/map_door_blocked|/room_prob_map_view|/obj_prob_map_view|/base_obj_prob_map_view|/base_room_prob_map_view|/sdf|/hierarchy" -O /media/thomas/EE702FC8702F967D/studium/Masterarbeit/output/result_test_we_' + str(p) + '.bag'
    proc_save = subprocess.Popen('xterm -e rosbag record -e "/base_obj(.*)|/base_room(.*)|/obj_(.*)|/room_(.*)|/occupied_cells_vis_array|/mapper_door_poses|/gmap|/map|/map_door_blocked|/room_prob_map_view|/obj_prob_map_view|/base_obj_prob_map_view|/base_room_prob_map_view|/sdf|/hierarchy" -O /media/thomas/EE702FC8702F967D/studium/Masterarbeit/output/result_test_we_' + str(p) + '.bag', shell=True)
    time.sleep(3)
    last_clock.clock = last_clock.clock + rospy.Duration(0.1)
    pub.publish(last_clock)
    time.sleep(3)

    proc_hierarchy = subprocess.Popen('xterm -e rosservice call /hierarchy_srv "debug_room: 0"', shell=True)
    proc_hierarchy.wait()
    time.sleep(5)
    terminate_ros_node("/record")

    time.sleep(3)
    terminate_process_and_children(proc_map)
    time.sleep(3)


