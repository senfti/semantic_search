import numpy as np

object_input_file = '/home/thomas/coco_data_objects.txt'
places_input_file = '/home/thomas/coco_places.txt'
label_file = "/home/thomas/BVLCcaffe/models/googlenet_places205/categories_places205.csv"
obj_label_file = "/home/thomas/darknet/data/coco.names"

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
names = []
reduced_place_obj_map = np.zeros((0,80), np.float64)
for i, c in enumerate(content):
    if c[0] != '_':
        print c
        names.append(c)
        reduced_place_obj_map = np.vstack((reduced_place_obj_map, place_obj_map[i]))

tmp = reduced_place_obj_map
print reduced_place_obj_map.shape[0], reduced_place_obj_map.shape[0]
reduced_place_obj_map = np.zeros((reduced_place_obj_map.shape[0],0), np.float64)
with open(obj_label_file) as f:
    content = f.readlines()
for i, c in enumerate(content):
    if c[0] != '_':
        print i, c
        names.append(c)
        reduced_place_obj_map = np.hstack((reduced_place_obj_map, tmp[:,[i]]))


print reduced_place_obj_map.shape
np.savetxt('/home/thomas/obj_places_map.dat', reduced_place_obj_map, delimiter=' ')


f = open('/home/thomas/obj_places_map.dat', 'r')
lines = f.readlines()
f.close()
f = open('/home/thomas/obj_places_map_view.csv', 'w')
for i, line in enumerate(lines):
    f.write(names[i].replace(" ", "") + ' ' + line)
f.close()