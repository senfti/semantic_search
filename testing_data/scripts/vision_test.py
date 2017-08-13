import roslib
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np
import h5py
import os
import sys
import argparse
import glob
import time
import cv2
import scipy.io

import caffe
import pyyolo
from thread import start_new_thread
import time

#input_file = '/home/thomas/Schreibtisch/test_images/uni/'

bridge = CvBridge()

darknet_path = '/home/thomas/pyyolo/darknet'
datacfg = 'cfg/coco.data'
cfgfile = 'cfg/yolo.cfg'
weightfile = '../yolo.weights'
thresh = 0.1
hier_thresh = 0.5

#default_prototxt = "/home/thomas/caffe/models/googlenet_places205/deploy_places205.protxt"
#default_model = "/home/thomas/caffe/models/googlenet_places205/googlelet_places205_train_iter_2400000.caffemodel"
#mean_file = ''
default_prototxt = "/home/thomas/caffe/models/placescnn/places205CNN_deploy.prototxt"
default_model = "/home/thomas/caffe/models/placescnn/places205CNN_iter_300000.caffemodel"
mean_file = "/home/thomas/caffe/models/placescnn/places_mean.mat"

classifier = None
label_names = None
is_classifying = False


def caffe_init():
    global mean_file, classifier, label_names
    pycaffe_dir = os.path.dirname(__file__)

    image_dims = [256,256]

    mean, channel_swap = None, None
    if mean_file.endswith('mat'):
        mean = scipy.io.loadmat(mean_file)['image_mean']
        mean = np.swapaxes(mean, 0,2)
        mean = mean.mean(1).mean(1)
    elif mean_file.endswith('npy'):
        mean = np.load(mean_file)
    channel_swap = [int(s) for s in '2,1,0'.split(',')]

    caffe.set_mode_gpu()

    # Make classifier.
    classifier = caffe.Classifier(os.path.join(pycaffe_dir, default_prototxt), os.path.join(pycaffe_dir, default_model),
            image_dims=image_dims, mean=mean, raw_scale=255.0, channel_swap=channel_swap)

    pyyolo.init(darknet_path, datacfg, cfgfile, weightfile)
    with open("/home/thomas/caffe/models/placescnn/categoryIndex_places205.csv") as f:
        label_names = f.readlines()

output_img = None

def classify(img):
    global classifier, label_names, is_classifying, output_img
    start = time.time()
    cv2.imwrite('/tmp/img.png', img)
    img_res = [cv2.resize(img, (224,224))/255.0]
    predictions = classifier.predict(img_res, True)
    order = np.argsort(predictions[0])

    output = pyyolo.test('/tmp/img.png', thresh, hier_thresh)

    # Output
    for j in range(len(output)):
        cv2.rectangle(img, (output[j]['left'], output[j]['top']),
                      (output[j]['right'], output[j]['bottom']),
                      (255, 255, 255), thickness=1)
        cv2.putText(img, "{0}:{1:.2}".format(output[j]['class'], output[j]['prob']),
                    (output[j]['left'], output[j]['top']), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255),
                    thickness=1)
    cv2.putText(img,'{}, {}'.format(label_names[order[-1]].split(' ')[0], predictions[0][order[-1]]),
                (0,20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0),
                thickness=1)

    for j in range(0, 5):
        print '{}: {}, {}'.format(j, label_names[order[-j - 1]].split(' ')[0], predictions[0][order[-j - 1]]),
    print '\n',
    for j in range(0, len(output)):
        print '{}: {}, {}'.format(j, output[j]['class'], output[j]['prob']),
    print '\n'
    output_img = img
    is_classifying = False
    print("Scene done in %.2f s." % (time.time() - start))


def callback(data):
    global bridge, is_classifying, output_img

    if is_classifying:
        return

    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    is_classifying = True
    start_new_thread(classify, (cv_image,))
    cv2.imshow('output', cv2.resize(output_img, (1280,960)))
    cv2.waitKey(1)


rospy.init_node('vision_test', anonymous=True)
rospy.Subscriber("/camera/rgb/image_raw", Image, callback, queue_size=1)
caffe_init()

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
