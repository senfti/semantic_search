import cv2
import numpy as np
import matplotlib as plt
import random

sdf = np.ones((300,1000,3), np.uint8)*255
cv2.imshow("sdf",sdf)
cv2.waitKey()

data1 = []
data2 = []
data3 = []
for i in range(10):
    data1.append(random.uniform(0.01, 0.1))
    if i<10:
        data2.append(random.uniform(0.9, 0.99))
    else:
        data2.append(random.uniform(0.01, 0.1))
    if i<200:
        data3.append(random.uniform(0.01, 0.05))
    else:

plt.
