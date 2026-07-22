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

files = ["/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/h5e.bag",
"/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/h6e.bag",
"/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/h7e.bag"]

for filename in files:
    folder = "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/"
    bag = rosbag.Bag(filename)
    filename = os.path.basename(filename)
    print filename

    fn = open("/home/thomas/Masterarbeit/output/explore/" + filename + ".txt", "w")
    action_msgs = bag.read_messages(topics="/hierarchy")
    for topic, msg, t in action_msgs:
        fn.write("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% "+str(msg.curr_room)+"\n")
        obj_sort = sorted(range(len(msg.rooms[msg.curr_room].obj_probs)),
                          key=lambda x: msg.rooms[msg.curr_room].obj_probs[x], reverse=True)
        for j in range(len(obj_sort)):
            print '{:22}'.format(obj_name[obj_sort[j]]), '{:6.4f}'.format(
                msg.rooms[msg.curr_room].obj_probs[obj_sort[j]])
            fn.write('{:22}'.format(obj_name[obj_sort[j]]) + '{:6.4f}'.format(
                msg.rooms[msg.curr_room].obj_probs[obj_sort[j]]) + "\n")
        print
        fn.write("\n")
        room_sort = sorted(range(len(msg.rooms[msg.curr_room].room_type_probs)),
                           key=lambda x: msg.rooms[msg.curr_room].room_type_probs[x], reverse=True)
        for j in range(len(room_sort)):
            print '{:22}'.format(room_name[room_sort[j]]), '{:6.4f}'.format(
                msg.rooms[msg.curr_room].room_type_probs[room_sort[j]])
            fn.write('{:22}'.format(room_name[room_sort[j]]) + '{:6.4f}'.format(
                msg.rooms[msg.curr_room].room_type_probs[room_sort[j]]) + "\n")
    fn.close()