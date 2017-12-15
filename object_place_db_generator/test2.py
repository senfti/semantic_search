import numpy as np

object_input_file = '/home/thomas/coco_data_objects.txt'
places_input_file = '/home/thomas/coco_places.txt'

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
print place_obj_map.size
#place_obj_map = np.vstack([place_obj_map, object_sum / len(lines)])

#place_obj_map = place_obj_map / sum(place_obj_map)

np.savetxt('/home/thomas/obj_places_map.dat', place_obj_map, delimiter=' ')