import numpy as np
import cv2

# c1 = [[255,255,255,255], [134,137,112,255], [77,77,77,255], [40,41,34,255] ]
# c2 = [[230,230,230,255], [255,255,255,0], [100,100,100,255], [100,100,100,255] ]
#
c1 = [[0,0,0,255], [255,255,255,255], [1,2,3,255]]
c2 = [[1,2,3,255], [230,230,230,255], [255,255,255,255]]

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
    b_channel, g_channel, r_channel = cv2.split(img)
    alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 255
    img_BGRA = cv2.merge((r_channel, g_channel, b_channel, alpha_channel))

    for i in range(len(c1)):
        img_BGRA[np.where((img_BGRA == c1[i]).all(axis=2))] = c2[i]
    cv2.imwrite(f[:-4] + '_b' + f[-4:],img_BGRA)