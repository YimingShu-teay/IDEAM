import math
import time
import sys
sys.path.append(r"C:\Users\sym02\Desktop\Research\Extension\codes\decision_change") 
import matplotlib.pyplot as plt
import numpy as np
from Control.utils import normalize_angle
from shapely.geometry import LineString,Polygon
import matplotlib.pyplot as plt

class Path:

    def __init__(self, l1, l2, r, traslx=0, trasly=0):
        self.l1 = l1
        self.l2 = l2
        self.r = r
        self.traslx = traslx
        self.trasly = trasly

    def __call__(self, s): #返回x与y的坐标轴
        s = float(s)
        completed_lap = 0
        #print(s)
        while s >= 2*self.l1 + 2*self.r*math.pi + 2*self.l2:
            completed_lap += 1 # 完成了一圈
            s = s - (2*self.l1 + 2*self.r*math.pi + 2*self.l2)
        while s<0: #不知道这是一种什么样的状况,可能初始的时刻范围给了负值
            s = s + (2*self.l1 + 2*self.r*math.pi + 2*self.l2)
        # first edge 第一段直线路径
        if s>=0 and s<self.l1:
            label_s = "edge1"
            return (s + self.traslx , 0. + self.trasly)
        # first arc  第一段弧线路径
        elif s>=self.l1 and s< self.l1 + 2*self.r*math.pi/4:
            label_s = "arc1"
            arc = s - self.l1
            alpha = arc/self.r
            return (self.l1 + self.r*math.sin(alpha) + self.traslx , self.r - self.r*math.cos(alpha) + self.trasly)
        # second edge 第二段直线路径
        elif s>= self.l1 + 2*self.r*math.pi/4 and s< self.l1 + 2*self.r*math.pi/4 + self.l2:
            label_s = "edge2"
            return (self.l1 + self.r  + self.traslx , s - self.l1 - 2*self.r*math.pi/4 + self.r + self.trasly)
        # second arc 第二段弧线路径
        elif s>= self.l1 + 2*self.r*math.pi/4 + self.l2 and s < self.l1 + self.r*math.pi + self.l2:
            label_s = "arc2"
            arc = s - (self.l1 + 2*self.r*math.pi/4 + self.l2)
            alpha = arc/self.r
            return (self.l1 + math.cos(alpha)*self.r + self.traslx , self.r + self.l2 + math.sin(alpha)*self.r + self.trasly)
        # third edge 第三段直线路径
        elif s>= self.l1 + self.r*math.pi + self.l2 and s < 2*self.l1 + self.r*math.pi + self.l2:
            label_s = "edge3"
            return (self.l1 - (s - self.l1 - self.r*math.pi - self.l2) + self.traslx , 2*self.r+self.l2 + self.trasly)
        # third arc
        elif s>= 2*self.l1 + self.r*math.pi + self.l2 and s< 2*self.l1 + self.r*math.pi + self.l2 + 2*self.r*math.pi/4:
            label_s = "arc3"
            arc = s - (2*self.l1 + self.r*math.pi + self.l2)
            alpha = arc/self.r
            return (-math.sin(alpha)*self.r + self.traslx , self.r+ self.l2+self.r*math.cos(alpha) + self.trasly)
        # fourth edge
        elif s>= 2*self.l1 + self.r*math.pi + self.l2 + 2*self.r*math.pi/4 and s< 2*self.l1 + self.r*math.pi + 2*self.l2 + 2*self.r*math.pi/4:
            label_s = "edge4"
            return (-self.r + self.traslx , self.r+ self.l2 - (s-(2*self.l1 + self.r*math.pi + self.l2 + 2*self.r*math.pi/4)) + self.trasly)
        # fourth arc
        elif s>= 2*self.l1 + self.r*math.pi + 2*self.l2 + 2*self.r*math.pi/4 and s< 2*self.l1 + 2*self.r*math.pi + 2*self.l2:
            label_s = "arc4"
            arc = s - (2*self.l1 + self.r*math.pi + 2*self.l2 + 2*self.r*math.pi/4)
            alpha = arc/self.r
            return (-self.r*math.cos(alpha) + self.traslx , self.r- self.r*math.sin(alpha) + self.trasly)
        else:
            Exception("Sbagliato controlla!")
    
    
    
    def start_end_dict(self,path_d):
        ind1 = np.where(path_d==0)[0][0]
        ind2 = np.where(path_d==self.l1)[0][0]
        ind3 = np.where((path_d==self.l1 + 2*self.r*math.pi/4)[0][0])
        ind4 = np.where((path_d==self.l1 + 2*self.r*math.pi/4 + self.l2)[0][0])
        ind5 = np.where((path_d==self.l1 + self.r*math.pi + self.l2)[0][0])
        ind6 = np.where((path_d==2*self.l1 + self.r*math.pi + self.l2)[0][0])
        ind7 = np.where((path_d==2*self.l1 + self.r*math.pi + self.l2 + 2*self.r*math.pi/4)[0][0])
        ind8 = np.where((path_d==2*self.l1 + self.r*math.pi + 2*self.l2 + 2*self.r*math.pi/4)[0][0])
        ind9 = np.where((path_d==2*self.l1 + 2*self.r*math.pi + 2*self.l2)[0][0])
                        
        SD_dict = {"edge1":[ind1,ind2],
                   "arc1":[ind2,ind3],
                   "edge2":[ind3,ind4],
                   "arc2":[ind4,ind5],
                   "edge3":[ind5,ind6],
                   "arc3":[ind6,ind7],
                   "edge4":[ind7,ind8],
                   "arc4":[ind8,ind9]}
        return SD_dict
    
    
    def get_track_frenet_coords(self, x_list, y_list, x0, path_d, ey_label, lane_width):
        
        '''
        输入：
        x0: 目前 ego vehicle 所处位置
        path_d: ego vehicle 所要追踪的车道中心线在 frenet 坐标系下的 s_list
        x_list, y_list: path_d 在笛卡尔坐标系下的坐标列表
        ey_label: 目前 ego vehicle 在 path_d 上 ey的正负标识符号
        lane_width:一条道路的宽度
        
        输出：
        s_now, ey_now: ego vehicle 位于 path_d 的坐标位置
        '''
        
        location, label_s =self.get_cartesian_coords(x0[3],x0[4])
        X_dict, _ = self.get_separated_start_end_XY(path_d,x_list,y_list)
        x_seg = X_dict[label_s]
        x_ind = np.where(x_seg == location[0])
        s_now = path_d[x_ind]
        if ey_label:
            ey_now = lane_width
        else:
            ey_now = -lane_width
        return s_now, ey_now
    
    
    def get_separated_start_end_XY(self,path_d,x_list,y_list):
        SD_dict = self.start_end_dict(path_d)
        X_dict = {}
        Y_dict = {}
        for key, value in SD_dict:
            X_dict[key] = x_list[SD_dict[key][0],SD_dict[key][1]]
            Y_dict[key] = y_list[SD_dict[key][0],SD_dict[key][1]]
        return X_dict, Y_dict
    

    def get_cartesian_coords(self, s, l):
        res= self(s)
        (x1, y1) = res
        theta_r = self.get_theta_r(s)
        x = x1 - np.sin(theta_r)*l
        y = y1 + np.cos(theta_r)*l
        return (x, y)

    def get_k(self, s):
        s = float(s)
        completed_lap = 0
        while s >= 2*self.l1 + 2*self.r*math.pi + 2*self.l2:
            completed_lap += 1
            s = s - (2*self.l1 + 2*self.r*math.pi + 2*self.l2)
        while s<0:
            s = s + (2*self.l1 + 2*self.r*math.pi + 2*self.l2)

        # first edge
        if s>=0 and s<self.l1:
            return 0.
        # first arc
        elif s>=self.l1 and s< self.l1 + 2*self.r*math.pi/4:
            return 1./self.r
        # second edge
        elif s>= self.l1 + 2*self.r*math.pi/4 and s< self.l1 + 2*self.r*math.pi/4 + self.l2:
            return 0.
        # second arc
        elif s>= self.l1 + 2*self.r*math.pi/4 + self.l2 and s < self.l1 + self.r*math.pi + self.l2:
            return 1./self.r
        # third edge
        elif s>= self.l1 + self.r*math.pi + self.l2 and s < 2*self.l1 + self.r*math.pi + self.l2:
            return 0.
        # third arc
        elif s>= 2*self.l1 + self.r*math.pi + self.l2 and s< 2*self.l1 + self.r*math.pi + self.l2 + 2*self.r*math.pi/4:
            return 1./self.r
        # fourth edge
        elif s>= 2*self.l1 + self.r*math.pi + self.l2 + 2*self.r*math.pi/4 and s< 2*self.l1 + self.r*math.pi + 2*self.l2 + 2*self.r*math.pi/4:
            return 0.
        # fourth arc
        elif s>= 2*self.l1 + self.r*math.pi + 2*self.l2 + 2*self.r*math.pi/4 and s< 2*self.l1 + 2*self.r*math.pi + 2*self.l2:
            return 1./self.r
        else:
            Exception("Sbagliato controlla!")

    def get_theta_r(self, s):
        s = float(s)
        completed_lap = 0
        while s >= 2*self.l1 + 2*self.r*math.pi + 2*self.l2:
            completed_lap += 1
            s = s - (2*self.l1 + 2*self.r*math.pi + 2*self.l2)
        while s < 0:
            s = s + (2*self.l1 + 2*self.r*math.pi + 2*self.l2)

        # first edge
        if s>=0 and s<self.l1:
            return normalize_angle(0.)
        # first arc
        elif s>=self.l1 and s< self.l1 + 2*self.r*math.pi/4:
            arc = s - self.l1
            alpha = arc/self.r
            return normalize_angle(alpha)
        # second edge
        elif s>= self.l1 + 2*self.r*math.pi/4 and s< self.l1 + 2*self.r*math.pi/4 + self.l2:
            return normalize_angle(math.pi/2)
        # second arc
        elif s>= self.l1 + 2*self.r*math.pi/4 + self.l2 and s < self.l1 + self.r*math.pi + self.l2:
            arc = s - (self.l1 + 2*self.r*math.pi/4 + self.l2)
            alpha = arc/self.r
            return normalize_angle(math.pi/2+alpha)
        # third edge
        elif s>= self.l1 + self.r*math.pi + self.l2 and s < 2*self.l1 + self.r*math.pi + self.l2:
            return normalize_angle(math.pi)
        # third arc
        elif s>= 2*self.l1 + self.r*math.pi + self.l2 and s< 2*self.l1 + self.r*math.pi + self.l2 + 2*self.r*math.pi/4:
            arc = s - (2*self.l1 + self.r*math.pi + self.l2)
            alpha = arc/self.r
            return normalize_angle(math.pi+alpha)
        # fourth edge
        elif s>= 2*self.l1 + self.r*math.pi + self.l2 + 2*self.r*math.pi/4 and s< 2*self.l1 + self.r*math.pi + 2*self.l2 + 2*self.r*math.pi/4:
            return normalize_angle((3/2)*math.pi)
        # fourth arc
        elif s>= 2*self.l1 + self.r*math.pi + 2*self.l2 + 2*self.r*math.pi/4 and s< 2*self.l1 + 2*self.r*math.pi + 2*self.l2:
            arc = s - (2*self.l1 + self.r*math.pi + 2*self.l2 + 2*self.r*math.pi/4)
            alpha = arc/self.r
            return normalize_angle((3/2)*math.pi + alpha)
        else:
            Exception("Sbagliato controlla!")

    def get_len(self):
        return 2*self.l1 + 2*self.r*math.pi + 2*self.l2


path = Path(200, 200, 100,-100,-200)
path1c = Path(200, 200, 101.75,-100,-201.75)
path1 = Path(200, 200, 103.5,-100,-203.5)
path2c = Path(200, 200, 105.25,-100,-205.25)
path2 = Path(200, 200, 107,-100,-207)
path3c = Path(200, 200, 108.75,-100,-208.75)
path3 = Path(200, 200, 110.5,-100,-210.5)

samples = np.arange(0., 1400, 0.01)
samples1c = np.arange(0., 1400., 0.01)
samples1 = np.arange(0., 1400., 0.01)
samples2c = np.arange(0., 1450., 0.01)
samples2 = np.arange(0., 1450., 0.01)
samples3c = np.arange(0., 1450., 0.01)
samples3 = np.arange(0., 1450., 0.01)

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

# time_now = time.time()
# def coordinate_mapping(x_list,y_list,sample,x0_g):
#     xy_stack = np.transpose(np.array([x_list,y_list])) - x0_g
#     d = np.linalg.norm(xy_stack,ord=2, axis=1)
#     min_index = np.argmin(d)
#     s_map = sample[min_index]
#     ey_map = d[min_index]
#     return s_map,ey_map

# s,ey = coordinate_mapping(x2c,y2c,samples2c,[380,210])
# time_end = time.time()
# print(time_end - time_now)

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
    
    

# plt.xlim((-400, 400))
# plt.ylim((-400, 400))

# plt.plot(x, y, 'b',linewidth=0.5)
# plt.plot(x1, y1, 'b',linewidth=0.5)
# plt.plot(x2, y2, 'b',linewidth=0.5)
# plt.plot(x3, y3, 'b',linewidth=0.5)
# plt.plot(x1c, y1c, 'b',linestyle = 'dashed',linewidth=0.5)
# plt.plot(x2c, y2c, 'b',linestyle = 'dashed',linewidth=0.5)
# plt.plot(x3c, y3c, 'b',linestyle = 'dashed',linewidth=0.5)
# plt.show()
# Let's now create shapely LineStrings for each path
# Define a function to close the loop by ensuring the first and last points are the same

# for i in range(len(samples2c)):
#     samples2c[i] = round(samples2c[i],2)
    
    # Now we'll use the provided numpy arrays to create and plot the paths with shapely and matplotlib.

# We have to import the required modules first
# Adjusting the plot based on the user's request for lighter colors and grey fills.

# Assuming the arrays x1, y1, etc. are the coordinates for the paths and centerlines

# Create shapely LineStrings for each path using the mock coordinates as placeholders
import numpy as np
from shapely.geometry import LineString
import matplotlib.pyplot as plt

# Assuming x1, y1, x2, y2, x3, y3, x1c, y1c, x2c, y2c, x3c, y3c are defined elsewhere

# line = LineString(np.column_stack((x, y)))
# line1 = LineString(np.column_stack((x1, y1)))
# line2 = LineString(np.column_stack((x2, y2)))
# line3 = LineString(np.column_stack((x3, y3)))
# line1c = LineString(np.column_stack((x1c, y1c)))
# line2c = LineString(np.column_stack((x2c, y2c)))
# line3c = LineString(np.column_stack((x3c, y3c)))

# # Creating the plot
# fig, ax = plt.subplots()

# # Plot the paths with a light blue color
# ax.plot(*line.xy, color='darkgray', linewidth=1)
# ax.plot(*line2.xy, color='darkgray', linewidth=1)
# ax.plot(*line3.xy, color='darkgray', linewidth=1)
# ax.plot(*line1.xy, color='darkgray', linewidth=1)
# # Plot the centerlines with a light green color and dashed lines
# ax.plot(*line1c.xy, color='lightgray', linewidth=0.6, linestyle='--')
# ax.plot(*line2c.xy, color='lightgray', linewidth=0.6, linestyle='--')
# ax.plot(*line3c.xy, color='lightgray', linewidth=0.6, linestyle='--')
# # 填充line3和line之间的区域


# # Setting labels and title
# ax.set_xlabel('X coordinate')
# ax.set_ylabel('Y coordinate')
# # ax.set_xlim(-50,50)
# # ax.set_ylim(-220,-180)
# # Set aspect ratio to equal for accurate representation of geometry
# ax.set_aspect('equal')
# # Display the plot
# plt.savefig(r"C:\Users\sym02\Desktop\Research\Extension\codes\decision_change_rear\figsave\round.png",dpi=600)
# plt.show()



# def wrap_for_matlab_2c():
#     path_d = Path(500, 500, 105.25,-250,-355.25)
#     samples_d = np.arange(0., 2670, 0.01)
#     coord_d = []
#     for s in samples_d:
#         coord_d += [path_d(s)]
#     x_d = np.array([c[0][0] for c in coord_d])
#     y_d = np.array([c[0][1] for c in coord_d])
#     for i in range(len(samples_d)):
#         samples_d[i] = round(samples_d[i],2)
#     return path_d, samples_d, x_d, y_d
def coordinate_remapping(path_d,x_list,y_list,sample,x0_g_v):
    xy_stack = np.transpose(np.array([x_list,y_list])) - x0_g_v
    
    d = np.linalg.norm(xy_stack,ord=2, axis=1)
    # print("d=",d)
    min_index = np.argmin(d)

    s_map = sample[min_index]
    ey_map = d[min_index]
    
    theta_r = path_d.get_theta_r(s_map)
    sign = (x0_g_v[1]-y_list[min_index])*np.cos(theta_r) - (x0_g_v[0]-x_list[min_index])*np.sin(theta_r)
    if sign > 0:
        pass
    elif sign <0:
        ey_map = -ey_map
    return s_map,ey_map

def path_to_path_proj(s,ey,path_now,path_d):

    path_now, _,_,_ = get_path_info(path_now)
    path_d, x_d, y_d,sample_d = get_path_info(path_d) 
    
    x, y = [], []
    for i in range(len(s)):
        global_pos = path_now.get_cartesian_coords(s[i],ey[i])
        x.append(global_pos[0])
        y.append(global_pos[1])
    
    s_proj, ey_proj = [], []
    for i in range(len(s)):
        si_proj, eyi_proj = coordinate_remapping(path_d,x_d,y_d,sample_d,[x[i],y[i]])
        s_proj.append(si_proj)
        ey_proj.append(eyi_proj)
    return s_proj, ey_proj



