import subprocess
import signal
import time
import rospy
import os
from rosgraph_msgs.msg import Clock
from subprocess import check_output

param = [
#         ["object_prior:=0.003", "object_hit:=0.04", "object_miss:=0.00001"],
#         ["object_prior:=0.003", "object_hit:=0.04", "object_miss:=0.00003"],
 #        ["object_prior:=0.003", "object_hit:=0.04", "object_miss:=0.00007"],
 ##        ["object_prior:=0.003", "object_hit:=0.04", "object_miss:=0.00012"],
    #     ["object_prior:=0.003", "object_hit:=0.04", "object_miss:=0.0002"],
 #        ["object_prior:=0.003", "object_hit:=0.08", "object_miss:=0.00001"],
  ##       ["object_prior:=0.003", "object_hit:=0.08", "object_miss:=0.00003"],
   ##      ["object_prior:=0.003", "object_hit:=0.08", "object_miss:=0.00007"],
    #     ["object_prior:=0.003", "object_hit:=0.08", "object_miss:=0.00012"],
    ##     ["object_prior:=0.003", "object_hit:=0.08", "object_miss:=0.0002"],
     #    ["object_prior:=0.003", "object_hit:=0.12", "object_miss:=0.00001"],
     ##    ["object_prior:=0.003", "object_hit:=0.12", "object_miss:=0.00003"],
     #    ["object_prior:=0.003", "object_hit:=0.12", "object_miss:=0.00007"],
      #   ["object_prior:=0.003", "object_hit:=0.12", "object_miss:=0.00012"],
      #   ["object_prior:=0.003", "object_hit:=0.12", "object_miss:=0.0002"],
      ##   ["object_prior:=0.003", "object_hit:=0.16", "object_miss:=0.00001"],
      #   ["object_prior:=0.003", "object_hit:=0.16", "object_miss:=0.00003"],
      #   ["object_prior:=0.003", "object_hit:=0.16", "object_miss:=0.00007"],
       #  ["object_prior:=0.003", "object_hit:=0.16", "object_miss:=0.00012"],
       #  ["object_prior:=0.003", "object_hit:=0.16", "object_miss:=0.0002"]
       #    ["object_prior:=0.003", "object_hit:=0.04", "object_miss:=0.0001"],
       #    ["object_prior:=0.003", "object_hit:=0.04", "object_miss:=0.0005"],
       #    ["object_prior:=0.003", "object_hit:=0.04", "object_miss:=0.002"],
       #    ["object_prior:=0.003", "object_hit:=0.1", "object_miss:=0.0001"],
       #    ["object_prior:=0.003", "object_hit:=0.1", "object_miss:=0.0005"],
       #    ["object_prior:=0.003", "object_hit:=0.1", "object_miss:=0.002"],
       #    ["object_prior:=0.003", "object_hit:=0.2", "object_miss:=0.0001"],
       #    ["object_prior:=0.003", "object_hit:=0.2", "object_miss:=0.0005"],
       #    ["object_prior:=0.003", "object_hit:=0.2", "object_miss:=0.002"],
       #    ["object_prior:=0.004", "object_hit:=0.04", "object_miss:=0.0001"],
       #    ["object_prior:=0.004", "object_hit:=0.04", "object_miss:=0.0005"],
       #    ["object_prior:=0.004", "object_hit:=0.04", "object_miss:=0.002"],
       #   ["object_prior:=0.004", "object_hit:=0.1", "object_miss:=0.0001"],
          ["object_prior:=0.004", "object_hit:=0.1", "object_miss:=0.0005"]
          # ["object_prior:=0.004", "object_hit:=0.1", "object_miss:=0.002"],
          # ["object_prior:=0.004", "object_hit:=0.2", "object_miss:=0.0001"],
          # ["object_prior:=0.004", "object_hit:=0.2", "object_miss:=0.0005"],
          # ["object_prior:=0.004", "object_hit:=0.2", "object_miss:=0.002"]
]


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
    print 'xterm -e roslaunch hardware semantic_mapping_v2_rosbag.launch ' + p[0] + ' ' + p[1] + ' ' + p[2]
    proc_map = subprocess.Popen('xterm -e roslaunch hardware semantic_mapping_v2_rosbag.launch ' + p[0] + ' ' + p[1] + ' ' + p[2], shell=True)
    time.sleep(10)

    proc_rosbag = subprocess.Popen('xterm -e rosbag play -u 650 --clock /media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/home_asus_5f.bag', shell=True)
    proc_rosbag.wait()

    proc_hierarchy = subprocess.Popen('xterm -e rosservice call /hierarchy_srv "debug_room: 0"', shell=True)
    proc_hierarchy.wait()

    print 'xterm -e rosbag record -e "/base_obj(.*)|/base_room(.*)|/obj_(.*)|/room_(.*)|/occupied_cells_vis_array|/mapper_door_poses|/gmap|/map|/map_door_blocked|/room_prob_map_view|/obj_prob_map_view|/base_obj_prob_map_view|/base_room_prob_map_view|/sdf|/hierarchy" -O /media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/output/result_test_rl1_' + p[0][14:] + '_' + p[1][12:] + '_' + p[2][13:] +'.bag'
    proc_save = subprocess.Popen('xterm -e rosbag record -e "/base_obj(.*)|/base_room(.*)|/obj_(.*)|/room_(.*)|/occupied_cells_vis_array|/mapper_door_poses|/gmap|/map|/map_door_blocked|/room_prob_map_view|/obj_prob_map_view|/base_obj_prob_map_view|/base_room_prob_map_view|/sdf|/hierarchy" -O /media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/output/result_test_rl1_' + p[0][14:] + '_' + p[1][12:] + '_' + p[2][13:] +'.bag', shell=True)
    time.sleep(5)
    last_clock.clock = last_clock.clock + rospy.Duration(0.1)
    pub.publish(last_clock)
    time.sleep(5)

    proc_hierarchy = subprocess.Popen('xterm -e rosservice call /hierarchy_srv "debug_room: 0"', shell=True)
    proc_hierarchy.wait()
    time.sleep(7)
    terminate_ros_node("/record")

    time.sleep(7)
    terminate_process_and_children(proc_map)
    time.sleep(3)

for p in param:
    print 'xterm -e roslaunch hardware semantic_mapping_v2_rosbag.launch ' + p[0] + ' ' + p[1] + ' ' + p[2]
    proc_map = subprocess.Popen('xterm -e roslaunch hardware semantic_mapping_v2_rosbag.launch ' + p[0] + ' ' + p[1] + ' ' + p[2], shell=True)
    time.sleep(5)

    proc_rosbag = subprocess.Popen('xterm -e rosbag play --clock /media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/home_asus_5f.bag', shell=True)
    proc_rosbag.wait()

    proc_hierarchy = subprocess.Popen('xterm -e rosservice call /hierarchy_srv "debug_room: 0"', shell=True)
    proc_hierarchy.wait()

    print 'xterm -e rosbag record -e "/base_obj(.*)|/base_room(.*)|/obj_(.*)|/room_(.*)|/occupied_cells_vis_array|/mapper_door_poses|/gmap|/map|/map_door_blocked|/room_prob_map_view|/obj_prob_map_view|/base_obj_prob_map_view|/base_room_prob_map_view|/sdf|/hierarchy" -O /media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/output/result_test_rl2_' + p[0][14:] + '_' + p[1][12:] + '_' + p[2][13:] +'.bag'
    proc_save = subprocess.Popen('xterm -e rosbag record -e "/base_obj(.*)|/base_room(.*)|/obj_(.*)|/room_(.*)|/occupied_cells_vis_array|/mapper_door_poses|/gmap|/map|/map_door_blocked|/room_prob_map_view|/obj_prob_map_view|/base_obj_prob_map_view|/base_room_prob_map_view|/sdf|/hierarchy" -O /media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/output/result_test_rl2_' + p[0][14:] + '_' + p[1][12:] + '_' + p[2][13:] +'.bag', shell=True)
    time.sleep(5)
    last_clock.clock = last_clock.clock + rospy.Duration(0.1)
    pub.publish(last_clock)
    time.sleep(5)

    proc_hierarchy = subprocess.Popen('xterm -e rosservice call /hierarchy_srv "debug_room: 0"', shell=True)
    proc_hierarchy.wait()
    time.sleep(7)
    terminate_ros_node("/record")

    time.sleep(7)
    terminate_process_and_children(proc_map)
    time.sleep(3)


# for p in param:
#     print 'xterm -e roslaunch hardware semantic_mapping_v2_rosbag.launch ' + p[0] + ' ' + p[1] + ' ' + p[2]
#     proc_map = subprocess.Popen('xterm -e roslaunch hardware semantic_mapping_v2_rosbag.launch ' + p[0] + ' ' + p[1] + ' ' + p[2], shell=True)
#     time.sleep(5)
#
#     proc_rosbag = subprocess.Popen('xterm -e rosbag play --clock /media/thomas/EE702FC8702F967D/studium/Masterarbeit/data/robolab_2.bag', shell=True)
#     proc_rosbag.wait()
#
#     proc_hierarchy = subprocess.Popen('xterm -e rosservice call /hierarchy_srv "debug_room: 0"', shell=True)
#     proc_hierarchy.wait()
#
#     print 'xterm -e rosbag record -e "/base_obj(.*)|/base_room(.*)|/obj_(.*)|/room_(.*)|/occupied_cells_vis_array|/mapper_door_poses|/gmap|/map|/map_door_blocked|/room_prob_map_view|/obj_prob_map_view|/base_obj_prob_map_view|/base_room_prob_map_view|/sdf|/hierarchy" -O /media/thomas/EE702FC8702F967D/studium/Masterarbeit/output/result_test_rl_' + p[0][14:] + '_' + p[1][12:] + '_' + p[2][13:] +'.bag'
#     proc_save = subprocess.Popen('xterm -e rosbag record -e "/base_obj(.*)|/base_room(.*)|/obj_(.*)|/room_(.*)|/occupied_cells_vis_array|/mapper_door_poses|/gmap|/map|/map_door_blocked|/room_prob_map_view|/obj_prob_map_view|/base_obj_prob_map_view|/base_room_prob_map_view|/sdf|/hierarchy" -O /media/thomas/EE702FC8702F967D/studium/Masterarbeit/output/result_test_rl_' + p[0][14:] + '_' + p[1][12:] + '_' + p[2][13:] +'.bag', shell=True)
#     time.sleep(5)
#     last_clock.clock = last_clock.clock + rospy.Duration(0.1)
#     pub.publish(last_clock)
#     time.sleep(5)
#
#     proc_hierarchy = subprocess.Popen('xterm -e rosservice call /hierarchy_srv "debug_room: 0"', shell=True)
#     proc_hierarchy.wait()
#     time.sleep(5)
#     terminate_ros_node("/record")
#
#     time.sleep(5)
#     terminate_process_and_children(proc_map)
#     time.sleep(3)

#
# for p in param:
#     print 'xterm -e roslaunch hardware semantic_mapping_v2_rosbag.launch ' + p[0] + ' ' + p[1] + ' ' + p[2]
#     proc_map = subprocess.Popen('xterm -e roslaunch hardware semantic_mapping_v2_rosbag.launch ' + p[0] + ' ' + p[1] + ' ' + p[2], shell=True)
#     time.sleep(5)
#
#     proc_rosbag = subprocess.Popen('xterm -e rosbag play --clock /media/thomas/EE702FC8702F967D/studium/Masterarbeit/data/home_asus_2.bag', shell=True)
#     proc_rosbag.wait()
#
#     proc_hierarchy = subprocess.Popen('xterm -e rosservice call /hierarchy_srv "debug_room: 0"', shell=True)
#     proc_hierarchy.wait()
#
#     print 'xterm -e rosbag record -e "/base_obj(.*)|/base_room(.*)|/obj_(.*)|/room_(.*)|/occupied_cells_vis_array|/mapper_door_poses|/gmap|/map|/map_door_blocked|/room_prob_map_view|/obj_prob_map_view|/base_obj_prob_map_view|/base_room_prob_map_view|/sdf|/hierarchy" -O /media/thomas/EE702FC8702F967D/studium/Masterarbeit/output/result_test_we_' + p[0][14:] + '_' + p[1][12:] + '_' + p[2][13:] +'.bag'
#     proc_save = subprocess.Popen('xterm -e rosbag record -e "/base_obj(.*)|/base_room(.*)|/obj_(.*)|/room_(.*)|/occupied_cells_vis_array|/mapper_door_poses|/gmap|/map|/map_door_blocked|/room_prob_map_view|/obj_prob_map_view|/base_obj_prob_map_view|/base_room_prob_map_view|/sdf|/hierarchy" -O /media/thomas/EE702FC8702F967D/studium/Masterarbeit/output/result_test_we_' + p[0][14:] + '_' + p[1][12:] + '_' + p[2][13:] +'.bag', shell=True)
#     time.sleep(5)
#     last_clock.clock = last_clock.clock + rospy.Duration(0.1)
#     pub.publish(last_clock)
#     time.sleep(5)
#
#     proc_hierarchy = subprocess.Popen('xterm -e rosservice call /hierarchy_srv "debug_room: 0"', shell=True)
#     proc_hierarchy.wait()
#     time.sleep(5)
#     terminate_ros_node("/record")
#
#     time.sleep(5)
#     terminate_process_and_children(proc_map)
#     time.sleep(3)
#
#
# for p in param:
#     print 'xterm -e roslaunch hardware semantic_mapping_v2_rosbag.launch ' + p[0] + ' ' + p[1] + ' ' + p[2]
#     proc_map = subprocess.Popen('xterm -e roslaunch hardware semantic_mapping_v2_rosbag.launch ' + p[0] + ' ' + p[1] + ' ' + p[2], shell=True)
#     time.sleep(5)
#
#     proc_rosbag = subprocess.Popen('xterm -e rosbag play --clock /media/thomas/EE702FC8702F967D/studium/Masterarbeit/data/home_asus_0.bag', shell=True)
#     proc_rosbag.wait()
#
#     proc_hierarchy = subprocess.Popen('xterm -e rosservice call /hierarchy_srv "debug_room: 0"', shell=True)
#     proc_hierarchy.wait()
#
#     print 'xterm -e rosbag record -e "/base_obj(.*)|/base_room(.*)|/obj_(.*)|/room_(.*)|/occupied_cells_vis_array|/mapper_door_poses|/gmap|/map|/map_door_blocked|/room_prob_map_view|/obj_prob_map_view|/base_obj_prob_map_view|/base_room_prob_map_view|/sdf|/hierarchy" -O /media/thomas/EE702FC8702F967D/studium/Masterarbeit/output/result_test_ws_' + p[0][14:] + '_' + p[1][12:] + '_' + p[2][13:] +'.bag'
#     proc_save = subprocess.Popen('xterm -e rosbag record -e "/base_obj(.*)|/base_room(.*)|/obj_(.*)|/room_(.*)|/occupied_cells_vis_array|/mapper_door_poses|/gmap|/map|/map_door_blocked|/room_prob_map_view|/obj_prob_map_view|/base_obj_prob_map_view|/base_room_prob_map_view|/sdf|/hierarchy" -O /media/thomas/EE702FC8702F967D/studium/Masterarbeit/output/result_test_ws_' + p[0][14:] + '_' + p[1][12:] + '_' + p[2][13:] +'.bag', shell=True)
#     time.sleep(5)
#     last_clock.clock = last_clock.clock + rospy.Duration(0.1)
#     pub.publish(last_clock)
#     time.sleep(5)
#
#     proc_hierarchy = subprocess.Popen('xterm -e rosservice call /hierarchy_srv "debug_room: 0"', shell=True)
#     proc_hierarchy.wait()
#     time.sleep(5)
#     terminate_ros_node("/record")
#
#     time.sleep(5)
#     terminate_process_and_children(proc_map)
#     time.sleep(3)
