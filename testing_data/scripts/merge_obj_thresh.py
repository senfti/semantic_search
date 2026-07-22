import numpy as np
import cv2
import os.path

contents = []
while True:
    try:
        line = raw_input("")
    except EOFError:
        break
    contents.append(line)


img = cv2.imread(contents[0])
img2 = cv2.imread(contents[1])
for x in xrange(img.shape[1]):
    for y in xrange(img.shape[0]):
        if img2[y,x,2] == 255 and img2[y,x,1] == 0:
            img[y,x,0] = 255
            img[y,x,1] = 0
            img[y,x,2] = 0
img2 = cv2.imread(contents[2])
for x in xrange(img.shape[1]):
    for y in xrange(img.shape[0]):
        if img2[y,x,2] == 255 and img2[y,x,1] == 0:
            img[y,x,0] = 0
            img[y,x,1] = 255
            img[y,x,2] = 0
img2 = cv2.imread(contents[3])
for x in xrange(img.shape[1]):
    for y in xrange(img.shape[0]):
        if img2[y,x,2] == 255 and img2[y,x,1] == 0:
            img[y,x,0] = 255
            img[y,x,1] = 255
            img[y,x,2] = 0

print os.path.dirname(contents[0])+"/"+"2_"+os.path.basename(contents[0])[1]+os.path.basename(contents[1])[1]+os.path.basename(contents[2])[1]+os.path.basename(contents[3])[1]+".png"
cv2.imwrite(os.path.dirname(contents[0])+"/"+"2_"+os.path.basename(contents[0])[1]+os.path.basename(contents[1])[1]+os.path.basename(contents[2])[1]+os.path.basename(contents[3])[1]+".png", img)
