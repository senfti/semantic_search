import numpy as np
import cv2

c1_ = []
c_ = [[1,1,1], [2,2,2], [3,3,3], [4,4,4], [5,5,5], [6,6,6], [7,7,7], [8,8,8], [9,9,9], [10,10,10], [11,11,11], [12,12,12], [13,13,13], [14,14,14]]
c2 = [[255,0,0], [179,0,0], [255,255,0], [179,179,0], [0,0,255], [0,0,179], [0,132,255], [0,93,179], [9,9,9], [10,10,10], [0,255,0], [0,179,0], [255,0,255], [179,0,179]]

contents = ["/home/thomas/semantic_search_git/semantic_search/Thesis/figures/wz_imgs/h5e/classes1_b.png"]
# while True:
#     try:
#         line = raw_input("")
#     except EOFError:
#         break
#     contents.append(line)

for f in contents:
    print f
    img = cv2.imread(f)

    for i in range(len(c1)):
        img[np.where((img == c1[i]).all(axis=2))] = c_[i]
        img[np.where((img == c_[i]).all(axis=2))] = c2[i]
    cv2.imwrite(f[:-4] + '_b' + f[-4:],img)