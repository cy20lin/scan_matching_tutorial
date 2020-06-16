import csv
import numpy as np
from matplotlib import pyplot as plt


before_csv = '../data/rectangle_points10.csv'
after_csv = '../data/rectangle_points10_rotated.csv'

# with open(before_csv, newline='') as csvfile:
#     rows = csv.reader(csvfile)
#     next(rows, None)  # skip the headers
#     before = np.array(rows)
before = np.loadtxt(before_csv, dtype=float, delimiter=',', skiprows=1)
after = np.loadtxt(after_csv, dtype=float, delimiter=',', skiprows=1)
print(before)
print(after)


fig = plt.figure("Points")
ax = fig.add_subplot()
ax.set_xlim(-5,5)
ax.set_ylim(-5,5)
ax.scatter(x=before[:,0],y=before[:,1],c='r')
ax.scatter(x=after[:,0],y=after[:,1],c='g')
plt.show()
