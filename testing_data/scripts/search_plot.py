import numpy as np
import matplotlib.pyplot as plt

f=open("/home/thomas/semantic_search_git/semantic_search/TestData/home_search.txt","r")

content = f.readlines()

timesf = []
timess = []
lengthf = []
lengths = []
times_searchf = []
times_searchs = []

i = 0
for l in content:
    w = l.split()
    if len(w) > 0:
        if i<48:
            if (i/3)%2 == 0:
                timesf.append(float(w[0]))
                times_searchf.append(float(w[3]))
            else:
                timess.append(float(w[0]))
                times_searchs.append(float(w[3]))

        if i>=48 and i<96:
            if (i/3)%2 == 0:
                lengthf.append(float(w[0]))
            else:
                lengths.append(float(w[0]))
        i+=1

f.close()
f=open("plot.txt","r")
plot1 = f.read()
f.close()
f=open("plot2.txt","r")
plot2 = f.read()
f.close()
f=open("plot3.txt","r")
plot3 = f.read()
f.close()
f=open("plot4.txt","r")
plot4 = f.read()
f.close()

out = plot1
idx=0
for t in timesf:
    out += "    (" + str(int(idx/3)+0.8) +", " + str(t) + ")[a]\n"
    idx+=1
idx=0
for t in timess:
    out += "    (" + str(int(idx/3)+1.2) +", " + str(t) + ")[b]\n"
    idx+=1
out+= plot2
for i in range(len(timesf)/3):
    v = (timesf[3*i]+timesf[3*i+1]+timesf[3*i+2])/3.0
    out += "    (" + str(int(i)+0.8) + ", " + str(v) + ")\n"
out+= plot3
for i in range(len(timess)/3):
    v = (timess[3*i]+timess[3*i+1]+timess[3*i+2])/3.0
    out += "    (" + str(int(i) + 1.2) + ", " + str(v) + ")\n"

out+= plot4

print out

out = plot1
idx=0
for t in lengthf:
    out += "    (" + str(int(idx/3)+0.8) +", " + str(t) + ")[a]\n"
    idx+=1
idx=0
for t in lengths:
    out += "    (" + str(int(idx/3)+1.2) +", " + str(t) + ")[b]\n"
    idx+=1
out+= plot2
for i in range(len(lengthf)/3):
    v = (lengthf[3*i]+lengthf[3*i+1]+lengthf[3*i+2])/3.0
    out += "    (" + str(int(i)+0.8) + ", " + str(v) + ")\n"
out+= plot3
for i in range(len(lengths)/3):
    v = (lengths[3*i]+lengths[3*i+1]+lengths[3*i+2])/3.0
    out += "    (" + str(int(i) + 1.2) + ", " + str(v) + ")\n"

out+= plot4


print out
