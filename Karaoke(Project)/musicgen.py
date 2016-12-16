str = "ch 2,ch 2,a 2,a 1,ch 1,dh 2,eh 2,dh 2,eh 2,g 2,g 2,g 1,a 1,b 2,a 4,- 4,a 2,c 2,d 2,c 1,e 1,d 2,e 2,d 2,c 2,g 4,- 4,ch 2,- 1,dh 1,eh 2,gh 2,ch 2,dh 1,dh 1,eh 2,gh 2,a 4,- 2,ch 2,b 4,- 4,ch 1,- 1,dh 1,eh 2,gh 2,ah 2,gh 2,eh 2,g 2,ch 2,ch 1,ch 1,a 2,eh 2,dh 4,- 4,ch 2,- 1,dh 1,eh 2,gh 2,ah 2,gh 2,eh 2,dh 2,a 4,- 2,ch 2,g 4,- 4,eh 2,- 1,gh 1,eh 2,gh 2,ah 2,gh 2,eh 2,dh 2,ch 4,a 2,ch 2,dh 4,- 1,eh 1,dh 1,ch 3,- 5"
k = str.split(",")
for i in k :
    x = i.split(" ");
    y = x[0]
    z = x[1]
    n = 0
    if y[0] == "-":
        n = 0
        #for j in range(0, int(z)) :
            #print(n)
        continue
    if y[0] == "c":
        n = 1
    if y[0] == "d":
        n = 3
    if y[0] == "e":
        n = 5
    if y[0] == "f":
        n = 6
    if y[0] == "g":
        n = 8
    if y[0] == "a":
        n = 10
    if y[0] == "b":
        n = 12
    if len(y) == 2 and y[1] == "h":
        n += 12
    if len(y) == 2 and y[1] == "+":
        n+=1
    for j in range(0, int(z)) :
        print(n)

