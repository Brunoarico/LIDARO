#!/usr/bin/python3

import sys
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math as m
from operator import itemgetter
import numpy as np
from scipy.stats import gaussian_kde
xx = []
yy = []
zz = []
pts= []

def sphe2cart (r, theta, phi):
    theta = np.deg2rad(theta)
    phi = np.deg2rad(phi)
    x = np.cos(theta) * np.sin(phi) * r
    y = np.sin(phi) * np.sin(theta) * r
    z = np.cos(phi) * r
    return x, y, z

def is_neighboor(pt, c, r):
     return pow((pt[0]-c[0]),2) + pow((pt[1]-c[1]),2) + pow((pt[2]-c[2]),2) < pow(r,2) 
    
def test_pt(list_pts, r, neigh):
    l_sorted_poits = sorted(list_pts, key=itemgetter(0))
    filt = []
    for i in l_sorted_poits:
        cont = 0
        for k in l_sorted_poits:
            if is_neighboor(k, i, r):
                cont = cont + 1
        if cont > neigh:
            filt.append(i)
    return filt

if(len(sys.argv) > 1):
    with open(sys.argv[1]) as File:
        reader = csv.reader(File, delimiter=',')
        for row in reader:
            print(row)
            a, b, c = float(row[1]), float(row[2]), float(row[3])
            xx.append(a)
            yy.append(b)
            zz.append(c)
            pts.append([a,b,c])


        xmin = ymin = zmin = -300
        xmax =  ymax =  zmax = +300
        if(len(sys.argv) > 2):
              filt = np.array(test_pt(pts, int(sys.argv[2]), int(sys.argv[3])))
        else:
              filt = np.array(test_pt(pts, 50, 5))

        xmin = min(filt[:,0]) 
        ymin = min(filt[:,1])
        zmin = min(filt[:,2])
        xmax = max(filt[:,0])  
        ymax = max(filt[:,1])
        zmax = max(filt[:,2])

        xyz = np.vstack([filt[:,0], filt[:,1], filt[:,2]])
        gauss = gaussian_kde(xyz)
        density = gauss(xyz)
        print(density)
        fig = plt.figure()
        ax = Axes3D(fig)
        ax.scatter(filt[:,0], filt[:,1], filt[:,2], c = density, lw =0, s = 20)
        
        #ax.scatter(xx, yy, zz, c = density, lw =0, s = 20)
        ax.set_xlim3d([xmin, xmax])
        ax.set_ylim3d([ymin, ymax])
        ax.set_zlim3d([zmin, zmax])             
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_ylabel("Z")
        ax.set_facecolor('xkcd:grey')
        ax.grid(False)
        ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.yaxis.set_ticks_position('bottom')
        ax.xaxis.set_ticks_position('bottom')
        ax.xaxis.set_ticks_position('bottom')
        
        
        plt.show()
        
else:
    print("Esqueceu nome do arquivo")
