import numpy as np
import math

n = 32
r = 11
center = n // 2
circle = np.zeros((n, n), dtype=int)

for i in range(24):
    theta = math.radians(i * 15)
    x = int(round(center + r * math.cos(theta)))
    y = int(round(center + r * math.sin(theta)))
    circle[y, x] = 1

for row in circle:
    print(list(row))
