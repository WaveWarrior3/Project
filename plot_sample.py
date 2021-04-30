import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d


f = open("points_x.txt", "r")
x = f.readlines()
f.close()
for i in range(len(x)):
    x[i] = x[i].replace("\n", "")
    x[i] = float(x[i])

f = open("points_y.txt", "r")
y = f.readlines()
f.close()
for i in range(len(y)):
    y[i] = y[i].replace("\n", "")
    y[i] = float(y[i])

f = open("points_z.txt", "r")
z = f.readlines()
f.close()
for i in range(len(z)):
    z[i] = z[i].replace("\n", "")
    z[i] = float(z[i])


fig = plt.figure()
ax = fig.gca(projection='3d')
ax.scatter(x, y, z)

plt.show()

print(x)
print(y)
print(z)