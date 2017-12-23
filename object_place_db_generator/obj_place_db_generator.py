import numpy as np

object_input_file = '/home/thomas/coco_data_objects.txt'
places_input_file = '/home/thomas/coco_places.txt'
label_file = "/home/thomas/BVLCcaffe/models/googlenet_places205/categories_places205.csv"

f = open(object_input_file, 'r')
lines = f.readlines()
objects = np.zeros((len(lines), 80), np.float64)
for i,l in enumerate(lines):
    tmp = l.split(' ')
    for obj in tmp[1:]:
        objects[i,int(obj)] = 1.0
f.close()

f = open(places_input_file, 'r')
lines = f.readlines()
places = np.zeros((len(lines), 205), np.float64)
for i,l in enumerate(lines):
    tmp = l.split(' ')
    for j in range(205):
        places[i,j] = float(tmp[j])
f.close()

place_sum = sum(places)
object_sum = sum(objects)

place_obj_map = np.dot(np.transpose(objects), places)

place_obj_map = np.transpose(place_obj_map / place_sum)
#place_obj_map = np.vstack([place_obj_map, object_sum / len(lines)])

#place_obj_map = place_obj_map / sum(place_obj_map)

with open(label_file) as f:
    content = f.readlines()
f.close()
content = [x.strip() for x in content]
reduced_place_obj_map = np.zeros((0,80), np.float64)
for i, c in enumerate(content):
    if c[0] != '_':
        print c
        reduced_place_obj_map = np.vstack((reduced_place_obj_map, place_obj_map[i]))

print reduced_place_obj_map.shape
np.savetxt('/home/thomas/obj_places_map.dat', reduced_place_obj_map, delimiter=' ')