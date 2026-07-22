import sys

names = ["art gallery","art studio","assembly line","attic","auditorium","ballroom","banquette hall","bar","basement",
         "beauty salon","bedroom","bookstore","bowling alley","butcher shop","bakery","cafeteria","candy store","classroom",
         "closet","clothe store","coffee shop","conference center","conference room","corridor","dining room","dorm room",
         "dinette","engine room","food court","galley","game room","gift shop","home office","hospital room","hotel room",
         "ice cream parlor","jail cell","kindergarden classroom","kitchen","kitchenette","laundromat","livingroom","lobby",
         "locker room","martial arts gym","music studio","nursery","office","pantry","parlor","reception","restaurant",
         "restaurant kitchen","shoe shop","shower","staircase","supermarket","television studio","veranda","waiting room"]

idx = 1
cols = 4

names = sorted(names)
n2 =[]
stride = len(names)/cols
for c in range(stride):
    for i in range(c,len(names),stride):
        n2.append(names[i])

print "\\begin{tabular}{|",
for i in range(cols):
    sys.stdout.write('l|')
print "} \\hline"
for x in n2:
    print x,
    if idx%cols == 0:
        print "\\\\ \\hline"
    else:
        print "& ",
    idx+=1