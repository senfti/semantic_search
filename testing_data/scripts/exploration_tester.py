import subprocess
import signal
import time
import rospy
import os
from rosgraph_msgs.msg import Clock
from subprocess import check_output
import rosbag
from std_msgs.msg import String
from semantic_mapping_v2.srv import HierarchySrv

last_clock = Clock()

def callback(data):
    global last_clock
    last_clock = data


proc_core = subprocess.Popen('xterm -e roscore', shell=True)
time.sleep(5)
rospy.init_node('mapping_test_script', anonymous=True)
rospy.Subscriber("/clock", Clock, callback, queue_size=1)
pub = rospy.Publisher('/clock', Clock, queue_size=1)
pub_img_save = rospy.Publisher('/save_all', String, queue_size=1)

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

room_name = ["art gallery", "art studio", "assembly line", "attic", "auditorium", "ballroom",
             "banquette hall",
             "bar", "basement", "beauty salon", "bedroom", "bookstore", "bowling alley", "butcher shop",
             "bakery",
             "cafeteria", "candy store", "classroom", "closet", "clothe store", "coffee shop",
             "conference center",
             "conference room", "corridor", "dining room", "dorm room", "dinette", "engine room",
             "food court",
             "galley", "game room", "gift shop", "home office", "hospital room", "hotel room",
             "ice cream parlor",
             "jail cell", "kindergarden classroom", "kitchen", "kitchenette", "laundromat", "livingroom",
             "lobby",
             "locker room", "martial arts gym", "music studio", "nursery", "office", "pantry", "parlor",
             "reception",
             "restaurant", "restaurant kitchen", "shoe shop", "shower", "staircase", "supermarket",
             "television studio", "veranda", "waiting room"]

files = ["h1e.bag", "h2e.bag", "h3e.bag", "h5e.bag", "h6e.bag", "h7e.bag"]

for i in range(len(files)):
    filename = files[i]
    folder = "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/"
    bag = rosbag.Bag(folder+filename)

    start_time = 0
    peek_times = []
    peek_rooms = []
    explore_times = []
    explore_rooms = []
    rooms = []
    room_times = []
    old = -1
    action_msgs = bag.read_messages(topics="/hierarchy")
    for topic, msg, t in action_msgs:
        rooms.append(msg.curr_room)
        room_times.append(t.to_sec())
        print "room: ", msg.curr_room, t.to_sec()

    action_msgs = bag.read_messages(topics="/current_action")
    for topic, msg, t in action_msgs:
        if start_time==0:
            start_time = t.to_sec()
        if (old==4 or old==5) and msg.data==-1:
            peek_times.append(t.to_sec())
            room_nr = 0
            for tr in range(len(room_times)):
                if room_times[tr] < t.to_sec()+5:
                    room_nr = rooms[tr]
            peek_rooms.append(room_nr)
            print "peek: ", t.to_sec(), room_nr

        old = msg.data

    action_msgs = bag.read_messages(topics="/room_explored")
    for topic, msg, t in action_msgs:
        explore_times.append(t.to_sec())
        room_nr = 0
        for tr in range(len(room_times)):
            if room_times[tr] < t.to_sec():
                room_nr = rooms[tr]
        explore_rooms.append(room_nr)
        print "explore: ", t.to_sec(), room_nr

    last_clock.clock = rospy.Time(0)

    print 'xterm -e roslaunch hardware semantic_mapping_v2_rosbag.launch'
    proc_map = subprocess.Popen('xterm -e roslaunch hardware semantic_mapping_v2_rosbag.launch', shell=True)

    time.sleep(10)
    proc_exec = subprocess.Popen('xterm -e roslaunch hardware execution_test.launch', shell=True)
    time.sleep(1)
    proc_view = subprocess.Popen('xterm -e rosrun prob_map_view prob_map_view', shell=True)
    time.sleep(1)

    peek_nr = 0
    explore_nr = 0
    old_stop = start_time
    while True:
        action = -1
        curr_room = 0
        next_stop = 9999999999
        if len(peek_times) > peek_nr:
            next_stop = peek_times[peek_nr]
            curr_room = peek_rooms[peek_nr]
            action = 0
        if len(explore_times) > explore_nr and explore_times[explore_nr] < next_stop:
            next_stop = explore_times[explore_nr]
            curr_room = explore_rooms[explore_nr]
            action = 1

        if action==-1:
            break

        print 'xterm -e rosbag play -s ' + str(old_stop-start_time) + ' -u ' + str(next_stop-old_stop) + ' --clock ' + folder+filename
        proc_rosbag = subprocess.Popen('xterm -e rosbag play -s ' + str(old_stop-start_time) + ' -u ' + str(next_stop-old_stop) + ' --clock ' + folder+filename, shell=True)
        proc_rosbag.wait()

        print 'xterm -e rosservice call /hierarchy_srv "debug_room: ' + str(curr_room) +'"'
        proc_hierarchy = subprocess.Popen('xterm -e rosservice call /hierarchy_srv "debug_room: ' + str(curr_room) +'"', shell=True)
        proc_hierarchy.wait()

        if action==0:
            peek_nr = peek_nr+1
            print 'xterm -e rosbag record -a -O /media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/output/' + filename + "_peek" + str(peek_nr) + '.bag'
            proc_save = subprocess.Popen('xterm -e rosbag record -a -O /media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/output/' + filename + "_peek" + str(peek_nr) + '.bag', shell=True)
        else:
            explore_nr = explore_nr+1
            print 'xterm -e rosbag record -a -O /media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/output/' + filename + "_explore" + str(explore_nr) + '.bag'
            proc_save = subprocess.Popen('xterm -e rosbag record -a -O /media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/output/' + filename + "_explore" + str(explore_nr) + '.bag', shell=True)

        time.sleep(5)
        last_clock.clock = last_clock.clock + rospy.Duration(0.1)
        pub.publish(last_clock)
        time.sleep(5)

        rospy.wait_for_service('hierarchy_srv')
        hierarchy_srv = rospy.ServiceProxy('hierarchy_srv', HierarchySrv)
        resp1 = hierarchy_srv(curr_room)
        time.sleep(5)
        terminate_ros_node("/record")

        save_string = String()
        if action==0:
            save_string.data = "/home/thomas/Masterarbeit/output/images/" + filename + "_peek" + str(peek_nr)
            text_file = open("/home/thomas/Masterarbeit/output/text/" + filename + "_peek" + str(peek_nr) + ".txt", "w")
            sorted_idx = sorted(range(len(resp1.rooms[curr_room].room_type_probs)), key=lambda x: resp1.rooms[curr_room].room_type_probs[x], reverse=True)
            text_file.write("Name - RoomRoom - Flat\n")
            for idx in sorted_idx:
                text_file.write('{:22}'.format(room_name[idx]) + "   " + '{:6.4f}'.format(resp1.rooms[curr_room].room_type_probs[idx]) + "    " + '{:6.4f}'.format(resp1.rooms[curr_room].room_type_probs_2[idx]) + "\n")
            text_file.close()
            pub_img_save.publish(save_string)
        else:
            save_string.data = "/home/thomas/Masterarbeit/output/images/" + filename + "_explore" + str(explore_nr)
            text_file = open("/home/thomas/Masterarbeit/output/text/" + filename + "_explore" + str(explore_nr) + ".txt", "w")
            sorted_idx = sorted(range(len(resp1.rooms[curr_room].room_type_probs)), key=lambda x: resp1.rooms[curr_room].room_type_probs[x], reverse=True)
            text_file.write("Name - RoomRoom - Flat\n")
            for idx in sorted_idx:
                text_file.write('{:22}'.format(room_name[idx]) + "   " + '{:6.4f}'.format(resp1.rooms[curr_room].room_type_probs[idx]) + "    " + '{:6.4f}'.format(resp1.rooms[curr_room].room_type_probs_2[idx]) + "\n")
            text_file.close()
            pub_img_save.publish(save_string)

        time.sleep(7)
        old_stop = next_stop+0.1

    terminate_process_and_children(proc_exec)
    time.sleep(1)
    terminate_process_and_children(proc_view)
    time.sleep(1)
    terminate_process_and_children(proc_map)
    time.sleep(7)

terminate_process_and_children(proc_core)
time.sleep(7)
