#!/usr/bin/env python
# license removed for brevity
import rospy
from vision.msg import VisionMsg
from vision.msg import DetectionSample
from vision.msg import ObjectDetectionMsg

def talker():
    pub = rospy.Publisher('vision_result', VisionMsg, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    msg = VisionMsg()
    msg.header.stamp = rospy.Time(1521801974)
    msg.header.frame_id = "/map"
    msg.objects.probs = []
    for i in range(61):
        msg.objects.probs.append(1)
    msg.objects.num_objects = 61
    msg.objects.num_boxes = 1
    msg.objects.samples = []
    sdf = DetectionSample()
    sdf.boxes = [0]
    sdf.point.x = -100
    sdf.point.y = -100
    sdf.point.z = -100
    msg.objects.samples.append(sdf)
    pub.publish(msg)
    rospy.Rate(0.5).sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass