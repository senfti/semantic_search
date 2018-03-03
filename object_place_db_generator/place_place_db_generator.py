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

param = 0.1

for i1,w1 in enumerate(places):
#    f.write(w1 + ' ')
    ss = []
    sum = 0.0
    #min = 0.0
    for i2,w2 in enumerate(places):
        try:
            s = max(spread.entry_named(w1, w2), 0.01)
            if i1 == i2:
                s = param
            else:
                sum += s
            ss.append(s)
            if(s < 0):
                print "lsdkfjsdlfkj%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
        except KeyError:
            print i, w1, w2
            i+=1
    print

    for i2,s in enumerate(ss):
        if i1 != i2:
            s = s / sum * (1-param)
        f.write(str(s) + ' ')
        print s,
    print
    f.write('\n')