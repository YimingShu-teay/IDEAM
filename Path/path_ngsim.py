import math
import time
import sys
sys.path.append("C:\\Users\\sym02\\Desktop\\Research\\Extension\\codes\\decision_improve") 
import matplotlib.pyplot as plt
import numpy as np
from Control.utils import normalize_angle
from Path.path import Path

path = Path(500, 0, 0,0,10.5)
path1c = Path(500, 0, 0,0,8.75)
path1 = Path(500, 0, 0,0,7)
path2c = Path(500, 0, 0,0,5.25)
path2 = Path(500, 0, 0,0,3.5)
path3c = Path(500, 0, 0,0,1.75)
path3 = Path(500, 0, 0,0,0)

samples = np.arange(0., 500, 0.01)
samples1c = np.arange(0., 500., 0.01)
samples1 = np.arange(0., 500., 0.01)
samples2c = np.arange(0., 500., 0.01)
samples2 = np.arange(0., 500., 0.01)
samples3c = np.arange(0., 500., 0.01)
samples3 = np.arange(0., 500., 0.01)

coord = []
coord1 = []
coord2 = []
coord3 = []
coord1c = []
coord2c = []
coord3c = []

for s in samples:
    coord += [path(s)]
for s in samples1:
    coord1 += [path1(s)]
for s in samples2:
    coord2 += [path2(s)]
for s in samples3:
    coord3 += [path3(s)]
    

for s in samples1c:
    coord1c += [path1c(s)]
for s in samples2c:
    coord2c += [path2c(s)]
for s in samples3c:
    coord3c += [path3c(s)]

    
x = np.array([c[0] for c in coord])
y = np.array([c[1] for c in coord])

x1 = np.array([c[0] for c in coord1])
y1 = np.array([c[1] for c in coord1])

x2 = np.array([c[0] for c in coord2])
y2 = np.array([c[1] for c in coord2])

x3 = np.array([c[0] for c in coord3])
y3 = np.array([c[1] for c in coord3])

x1c = np.array([c[0] for c in coord1c])
y1c = np.array([c[1] for c in coord1c])

x2c = np.array([c[0] for c in coord2c])
y2c = np.array([c[1] for c in coord2c])

x3c = np.array([c[0] for c in coord3c])
y3c = np.array([c[1] for c in coord3c])



plt.xlim((-400, 400))
plt.ylim((-25, 25))

# plt.plot(x, y, 'b',linewidth=0.5)
# plt.plot(x1, y1, 'b',linewidth=0.5)
# plt.plot(x2, y2, 'b',linewidth=0.5)
# plt.plot(x3, y3, 'b',linewidth=0.5)
# plt.plot(x1c, y1c, 'b',linestyle = 'dashed',linewidth=0.5)
# plt.plot(x2c, y2c, 'b',linestyle = 'dashed',linewidth=0.5)
# plt.plot(x3c, y3c, 'b',linestyle = 'dashed',linewidth=0.5)
# plt.show()
def plot_env():
    plt.plot(x, y, 'b',linewidth=0.8,zorder=1,color="black")
    plt.plot(x1, y1, 'b',linewidth=0.8,zorder=1,color="black")
    plt.plot(x2, y2, 'b',linewidth=0.8,zorder=1,color="black")
    plt.plot(x3, y3, 'b',linewidth=0.8,zorder=1,color="black")
    plt.plot(x1c, y1c, 'b',linestyle = 'dashed',linewidth=0.8,zorder=1,color="black")
    plt.plot(x2c, y2c, 'b',linestyle = 'dashed',linewidth=0.8,zorder=1,color="black")
    plt.plot(x3c, y3c, 'b',linestyle = 'dashed',linewidth=0.8,zorder=1,color="black")
    
    
def get_path_info(path_dindex):
    if path_dindex == 0:
        return path1c, x1c, y1c, samples1c
    if path_dindex == 1:
        return path2c, x2c, y2c, samples2c
    if path_dindex == 2:
        return path3c, x3c, y3c, samples3c