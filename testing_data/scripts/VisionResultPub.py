import roslib
import rospy
from vision.msg import VisionMsg
import glob
import pickle
import os
import operator

messages = []

for fn in sorted(glob.glob("/home/thomas/messages/*.dat"), key=os.path.getmtime):
    print fn
    messages.append(pickle.load(open(fn)))

rospy.init_node('vision_test', anonymous=True)
pub = rospy.Publisher('/vision_result', VisionMsg, queue_size=1)

for m in messages:
    if raw_input("Enter, e for exit        --------------------------------------------------------------------------------------------") == 'e':
        break
    pub.publish(m)
    m.place_guesses.sort(key=operator.attrgetter('prob'))
    print '\n'
    op = []
    for o in m.objects:
        op.append(o.prob[o.id])
    idx = sorted((e,i) for i,e in enumerate(op))
    for i in reversed(idx):
        print m.objects[i[1]].name, m.objects[i[1]].prob[m.objects[i[1]].id]
    print '\n'
    print m.place_guesses[-1].name, m.place_guesses[-1].prob, '\n', \
        m.place_guesses[-2].name, m.place_guesses[-2].prob, '\n', \
        m.place_guesses[-3].name, m.place_guesses[-3].prob

    print '\n'