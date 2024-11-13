from HITACHI_LDS360_LIDAR import HITACHI_LDS360
import time

import matplotlib.pyplot as plt
from collections import deque


LDS360=HITACHI_LDS360()

LDS360.start()

while not LDS360.dataIsAvailable():
    # could put a timeout here
    pass

plt.figure(figsize=(5,5))

X=[0]*360
Y=[0]*360

while True:
    try:
        points=LDS360.getDistancePoints()

        if len(points)>0:
            for p in range(360):
                (x,y) = points[p]
                X[p]=x
                Y[p]=y

            plt.scatter(X,Y)
            plt.draw()
            plt.pause(0.1)
            plt.clf()

        else:
            print(f"waiting for points got {len(points)}")

    except KeyboardInterrupt:
        pass

print("Stopping")
plt.close()
LDS360.stop()
distances=LDS360.getDistances()
with open("distances.dat","w") as f:
    for a in range(360):
        f.write(f"{a} {distances[a]}\n")
