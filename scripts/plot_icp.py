import imageio as io
import os
import csv
import numpy as np
from matplotlib import pyplot as plt

before_csv = './data/rectangle_points10.csv'
after_csv = './data/rectangle_points10_rotated.csv'
before = np.loadtxt(before_csv, dtype=float, delimiter=',', skiprows=1)
after = np.loadtxt(after_csv, dtype=float, delimiter=',', skiprows=1)
base_name = 'icp_iteration_'
ext = '.csv'
for i in range(11):
    name = base_name + str(i) + ext
    path = os.path.join('./data', name)
    data = np.loadtxt(path, dtype=float, delimiter=',', skiprows=1)
    fig = plt.figure("Points")
    ax = fig.add_subplot()
    ax.set_title('ICP iteration {}'.format(i))
    ax.set_xlim(-5,5)
    ax.set_ylim(-5,5)
    ax.scatter(x=before[:,0],y=before[:,1],c='g')
    ax.scatter(x=after[:,0],y=after[:,1],c='r')
    ax.scatter(x=data[:,0], y=data[:,1], c='k')
    output_path= os.path.join('./data', base_name + str(i) + '.png')
    plt.savefig(output_path)
    plt.close()

images = []
for i in range(11):
    image_path= os.path.join('./data', base_name + str(i) + '.png')
    images.append(io.imread(image_path))

output_path = os.path.join('./data', 'icp.gif')
io.mimsave(output_path, images, duration=0.5)
