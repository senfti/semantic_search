import urllib2
import threading
import time
import numpy as np
import flickrapi as f
import os.path

FLICKR_PUBLIC = 'dba2f9ff871aebbff8d2b8afc8db89e8'
FLICKR_SECRET = 'ee774a650917e423'
CSV_FILENAME = "/home/thomas/test.csv"

counter = 0

def getCount(tag, flickr):
    while 1:
        try:
            res = flickr.photos.search(tags=tag, tag_mode='all', media='photos', per_page='1')
            return res['photos']['total']
        except Exception as e:
            print tag, ': ', e



def getCountOfPlace(place_idx):
    global counter, place_object_counts, places, objects
    flickr = f.FlickrAPI(FLICKR_PUBLIC, FLICKR_SECRET, format='parsed-json')
    place = places[place_idx]
    if place == 'no_place':
        place_part = ''
    else:
        place_part = place + ','

    object_counts = np.zeros((1, len(objects)), np.int32)
    for o in range(len(objects)):
        obj = objects[o]
        if obj == 'no_object':
            if place_part == '':
                count = 0
            else:
                count = getCount(place, flickr)
        else:
            count = getCount(place_part + obj, flickr)
        object_counts[0,o] = count
        print counter
        counter+=1

    object_count_lock.acquire()
    place_object_counts[place_idx,:] = object_counts[0,:]
    object_count_lock.release()



class myThread (threading.Thread):
   def __init__(self, place_idx):
      threading.Thread.__init__(self)
      self.place_idx = place_idx
   def run(self):
       getCountOfPlace(self.place_idx)


places = open('/home/thomas/BVLCcaffe/models/placescnn/categories_places205.csv', 'r').read().splitlines()
places.append('no_place')
objects = open('/home/thomas/darknet/data/coco.names', 'r').read().splitlines()
objects.append('no_object')

start = time.time()
if os.path.isfile(CSV_FILENAME):
    place_object_counts = np.loadtxt(CSV_FILENAME, np.int32, delimiter=',')
else:
    place_object_counts = np.zeros((len(places), len(objects)), np.int32)
    place_count_lock = threading.Lock()
    object_count_lock = threading.Lock()
    threads = []

    for p in range(len(places)):
        thread = myThread(p)
        thread.start()
        threads.append(thread)

    # Wait for all threads to complete
    for t in threads:
        t.join()

    end = time.time()
    print 'Finished flickr in ', (end - start)

    np.savetxt(CSV_FILENAME, place_object_counts, delimiter=",", fmt='%d')

per_out = ','
for o in objects:
    if o == 'no_object':
        per_out += 'any object\n'
    else:
        per_out += o + ','

place_object_counts = np.sqrt(np.sqrt(place_object_counts))













for i in range(len(places)-1):
    per_out += places[i] + ','
    if place_object_counts[i,-1] == 0:
        place_object_counts[i, -1] = 1
    for o in range(len(place_object_counts[i,:])):
        if o == len(place_object_counts[i,:])-1:
            per_out += "{:.9f}".format(float(place_object_counts[i,-1]) / place_object_counts[i,-1]) + '\n'
        else:
            per_out += "{:.9f}".format(float(place_object_counts[i,o]) / place_object_counts[i,-1]) + ','

f = open('/home/thomas/test_percent.csv', 'w')
f.write(per_out)

end = time.time()
print 'Finished in', (end - start)
