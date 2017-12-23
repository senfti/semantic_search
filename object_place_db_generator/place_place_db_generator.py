import divisi2

assoc = divisi2.network.conceptnet_assoc('en')
U, S, _ = assoc.svd(k=100)
spread = divisi2.reconstruct_activation(U, S)

with open("/home/thomas/BVLCcaffe/models/googlenet_places205/categories_places205.csv") as f:
    content = f.readlines()
f.close()
content = [x.strip() for x in content]
places = []
for c in content:
    if c[0] != '_':
        places.append(c)

f = open('/home/thomas/room_spread.dat', 'w')
i = 1
#f.write(' ')
#for w2 in content:
#    f.write(w2 + ' ')
#f.write('\n')

for w1 in places:
#    f.write(w1 + ' ')
    ss = []
    sum = 0.0
    #min = 0.0
    for w2 in places:
        try:
            s = spread.entry_named(w1, w2)+0.1
            ss.append(s)
            sum += s
            if(s < 0):
                print "lsdkfjsdlfkj"
            #if s<min:
            #    min = s
        except KeyError:
            print i, w1, w2
            i+=1

    #sum += len(content)*min

    for s in ss:
        f.write(str(s/sum) + ' ')
    f.write('\n')