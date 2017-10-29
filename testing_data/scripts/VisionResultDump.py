import roslib
import rospy
from vision.msg import VisionMsg
import pickle


i=0
def callback(data):
    global i
    f = open('/home/thomas/messages/' + str(i) + '.dat', 'w')
    pickle.dump(data, f)
    i += 1


rospy.init_node('vision_test', anonymous=True)
rospy.Subscriber("/vision_result", VisionMsg, callback, queue_size=1)

rospy.spin()


