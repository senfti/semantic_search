import numpy as np
import cv2

contents = []
while True:
    try:
        line = raw_input("")
    except EOFError:
        break
    contents.append(line)

for f in contents:
    print f
    img = cv2.imread(f)
    img = img[40:380,10:350]
    rows,cols,_ = img.shape
    M = cv2.getRotationMatrix2D((cols/2,rows/2),103,1)
    img = cv2.warpAffine(img,M,(cols,rows), borderValue=[255,255,255])
    #img[np.where((img == [0,0,0]).all(axis=2))] = [255,255,255]
    cv2.imwrite(f[:-4] + '_b2' + f[-4:],img)