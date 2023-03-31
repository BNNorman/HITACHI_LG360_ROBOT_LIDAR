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

while True:
    try:
        points=LDS360.getAllAnglePoints()

        if len(points)>0:


            plt.scatter(*zip(*points))
            plt.draw()
            plt.pause(0.1)
            plt.clf()

        else:
            print(f"waiting for points go {len(points)}")

    except KeyboardInterrupt:
        pass
print("Stopped")
LDS360.stop()