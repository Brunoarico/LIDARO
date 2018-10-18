import csv
import numpy as np
import math as m
from operator import itemgetter
import numpy as np
from scipy import stats
from mayavi import mlab

xx = []
yy = []
zz = []
pts= []

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

    
with open('example.csv') as File:
    reader = csv.reader(File, delimiter=',')
    for row in reader:
        #print(row)
        a, b, c = float(row[1]), float(row[2]), float(row[3])
        xx.append(a)
        yy.append(b)
        zz.append(c)
        pts.append([a,b,c])
        
xyz = np.vstack([xx,yy,zz])
kde = stats.gaussian_kde(xyz)
density = kde(xyz)
print (density)
figure = mlab.figure('DensityPlot')
pts = mlab.points3d(xx, yy, zz, scale_factor=6)

#mlab.axes()
mlab.show()
