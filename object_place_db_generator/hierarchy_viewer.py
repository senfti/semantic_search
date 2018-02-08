import rospy
from std_msgs.msg import String
from semantic_mapping_v2.srv import HierarchySrvResponse

obj_name = ["person",  "bicycle",  "car",  "motorbike",  "aeroplane",  "bus",  "train",  "truck",  "boat",  "traffic light",
      "fire hydrant",  "stop sign",  "park meter",  "bench",  "bird",  "cat",  "dog",  "horse",  "sheep",  "cow",  "elephant",
      "bear",  "zebra",  "giraffe",  "backpack",  "umbrella",  "handbag",  "tie",  "suitcase",  "frisbee",  "ski",
      "snowboard",  "sport ball",  "kite",  "baseball bat",  "glove",  "skateboard",  "surf board",  "racket",
      "bottle",  "wine glass",  "cup",  "fork",  "knife",  "spoon",  "bowl",  "banana",  "apple",  "sandwich",  "orange",
      "broccoli",  "carrot",  "hot dog",  "pizza",  "donut",  "cake",  "chair",  "sofa",  "pot plant",  "bed",  "table",
      "toilet",  "monitor",  "laptop",  "mouse",  "remote",  "keyboard",  "cell phone",  "microwave",  "oven",  "toaster",
      "sink",  "refrigerator",  "book",  "clock",  "vase",  "scissor",  "teddy bear",  "hair dryer",  "toothbrush"]

room_name = ["art gallery", "art studio", "assembly line", "attic", "auditorium", "building", "ballroom", "hall",
      "bar", "basement", "beauty salon", "bedroom", "bookstore", "botanical garden", "bowl alley", "building",
      "butcher shop", "bakery", "cafeteria", "candy store", "hut", "classroom", "closet", "clothe store",
      "coffee shop", "conference auditorium", "business office", "corridor", "courthouse", "courtyard",
      "dine room", "dorm room", "dinner table", "doorway", "control room", "fire exit", "food court",
      "fountain", "galley", "game room", "gift shop", "home office", "hospital", "hospital", "hotel room",
      "hotel", "store ice cream", "inn", "prison cell", "kindergarden", "kitchen", "kitchenette", "laundromat",
      "livingroom", "lobby", "locker room", "mansion", "gym", "motel", "music studio", "museum", "nursery",
      "office", "office building", "pantry", "parlor", "patio", "reception", "restaurant", "restaurant"
      "kitchen", "patio", "school house", "shoe shop", "shop", "shower", "staircase", "supermarket",
      "stage", "pool", "television studio", "patio", "lounge"]

def callback(data):
    for i in range(len(data.rooms)):
        obj_sort = sorted(range(len(data.rooms[i].obj_probs)), key=lambda x: data.rooms[i].obj_probs[x], reverse=True)
        for j in range(len(obj_sort)):
            print obj_sort[j], '{:22}'.format(obj_name[obj_sort[j]]), '{:6.4f}'.format(data.rooms[i].obj_probs[obj_sort[j]])
        print

    obj = int(input("Object: "))
    print data.curr_room
    for i in range(len(data.rooms)):
        print '{:6.4f}'.format(data.rooms[i].obj_probs[obj]), '{:6.4f}'.format(data.rooms[i].single_view_obj_probs[obj]), '{:6.4f}'.format(data.rooms[i].obj_probs[obj]),
        print '{:9.4f}'.format(data.rooms[i].search_time), '{:9.4f}'.format(data.rooms[i].single_view_search_time)

    for i in range(len(data.links)):
        print data.links[i].room1, data.links[i].room2

def listener():
    rospy.init_node('hierarchy_viewer', anonymous=True)
    rospy.Subscriber("hierarchy", HierarchySrvResponse, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()