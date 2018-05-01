import rospy
import os
import rosbag
from nav_msgs.msg import Path
import tf
import math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

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


obj_name = ["person",  "bicycle",  "bird",  "cat",  "dog",  "backpack",  "umbrella",  "handbag",  "tie",  "suitcase",  "frisbee",  "ski",
      "snowboard",  "sport ball",  "kite",  "baseball bat",  "glove",  "skateboard",  "surf board",  "racket",
      "bottle",  "wine glass",  "cup",  "fork",  "knife",  "spoon",  "bowl",  "banana",  "apple",  "sandwich",  "orange",
      "broccoli",  "carrot",  "hot dog",  "pizza",  "donut",  "cake",  "chair",  "sofa",  "pot plant",  "bed",  "table",
      "toilet",  "monitor",  "laptop",  "mouse",  "remote",  "keyboard",  "cell phone",  "microwave",  "oven",  "toaster",
      "sink",  "refrigerator",  "book",  "clock",  "vase",  "scissor",  "teddy bear",  "hair dryer",  "toothbrush"]

room_name = ["art gallery","art studio","assembly line","attic","auditorium","ballroom","banquette hall",
             "bar","basement","beauty salon","bedroom","bookstore","bowling alley","butcher shop","bakery",
             "cafeteria","candy store","classroom","closet","clothe store","coffee shop","conference center",
             "conference room","corridor","dining room","dorm room","dinette","engine room","food court",
             "galley","game room","gift shop","home office","hospital room","hotel room","ice cream parlor",
             "jail cell","kindergarden classroom","kitchen","kitchenette","laundromat","livingroom","lobby",
             "locker room","martial arts gym","music studio","nursery","office","pantry","parlor","reception",
             "restaurant","restaurant kitchen","shoe shop","shower","staircase","supermarket",
             "television studio","veranda","waiting room"]

files = ["/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/explore_lab_1.bag",
"/home/thomas/Desktop/IST_expolere_su.bag"]

for filename in files:
    folder = "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/"
    bag = rosbag.Bag(filename)
    filename = os.path.basename(filename)
    print filename

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
        if start_time == 0:
            start_time = t.to_sec()
        if (old == -1) and (msg.data == 4 or msg.data == 5):
            peek_times.append(t.to_sec())
            room_nr = 0
            for tr in range(len(room_times)):
                if room_times[tr] < t.to_sec() + 5:
                    room_nr = rooms[tr]
            peek_rooms.append(room_nr)
            print "peek: ", t.to_sec()-start_time, room_nr

        old = msg.data

    hier_msgs = bag.read_messages(topics="/hierarchy")
    img_msgs = bag.read_messages(topics="/camera/rgb/image_raw")
    for t in peek_times:
        fn = open("/home/thomas/Masterarbeit/output/peek/" + filename + str(int(t-start_time)) + ".txt","w")
        for topic, msg, t2 in hier_msgs:
            if t2.to_sec() > t+10:
                obj_sort = sorted(range(len(msg.rooms[msg.curr_room].obj_probs)), key=lambda x: msg.rooms[msg.curr_room].obj_probs[x], reverse=True)
                for j in range(len(obj_sort)):
                    print '{:22}'.format(obj_name[obj_sort[j]]), '{:6.4f}'.format(msg.rooms[msg.curr_room].obj_probs[obj_sort[j]])
                    fn.write('{:22}'.format(obj_name[obj_sort[j]]) + '{:6.4f}'.format(msg.rooms[msg.curr_room].obj_probs[obj_sort[j]]) + "\n")
                print
                fn.write("\n")
                room_sort = sorted(range(len(msg.rooms[msg.curr_room].room_type_probs)), key=lambda x: msg.rooms[msg.curr_room].room_type_probs[x], reverse=True)
                for j in range(len(room_sort)):
                    print '{:22}'.format(room_name[room_sort[j]]), '{:6.4f}'.format(msg.rooms[msg.curr_room].room_type_probs[room_sort[j]])
                    fn.write('{:22}'.format(room_name[room_sort[j]]) + '{:6.4f}'.format(msg.rooms[msg.curr_room].room_type_probs[room_sort[j]]) + "\n")
                fn.close()
                break

        for topic, msg, t2 in img_msgs:
            if t2.to_sec() > t:
                print t-t2.to_sec(), "\n"
                try:
                    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                except CvBridgeError as e:
                    print(e)

                cv2.imwrite("/home/thomas/Masterarbeit/output/peek/" + filename + str(int(t-start_time)) + ".png", cv_image)
                break
        fn.close()
