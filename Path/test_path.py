import numpy as np
import math
import matplotlib.pyplot as plt


class MyReferencePath:
    def __init__(self):
        self.refer_path = np.zeros((60000, 6))
        self.refer_path[:,0] = np.linspace(0, 600, 60000) # x
        self.refer_path[:,1] = 2*np.sin(self.refer_path[:,0]/12.0) # y
        for i in range(len(self.refer_path)):
            if i == 0:
                dx = self.refer_path[i+1,0] - self.refer_path[i,0]
                dy = self.refer_path[i+1,1] - self.refer_path[i,1]
                ddx = self.refer_path[2,0] + self.refer_path[0,0] - 2*self.refer_path[1,0]
                ddy = self.refer_path[2,1] + self.refer_path[0,1] - 2*self.refer_path[1,1]
                ds = 0
            elif i == (len(self.refer_path)-1):
                dx = self.refer_path[i,0] - self.refer_path[i-1,0]
                dy = self.refer_path[i,1] - self.refer_path[i-1,1]
                ddx = self.refer_path[i,0] + self.refer_path[i-2,0] - 2*self.refer_path[i-1,0]
                ddy = self.refer_path[i,1] + self.refer_path[i-2,1] - 2*self.refer_path[i-1,1]
                dx_s = self.refer_path[i,0] - self.refer_path[i-1,0]
                dy_s = self.refer_path[i,1] - self.refer_path[i-1,1]
                ds = np.sqrt(dx_s**2+dy_s**2)
            else:      
                dx = self.refer_path[i+1,0] - self.refer_path[i,0]
                dy = self.refer_path[i+1,1] - self.refer_path[i,1]
                ddx = self.refer_path[i+1,0] + self.refer_path[i-1,0] - 2*self.refer_path[i,0]
                ddy = self.refer_path[i+1,1] + self.refer_path[i-1,1] - 2*self.refer_path[i,1]
                
                dx_s = self.refer_path[i,0] - self.refer_path[i-1,0]
                dy_s = self.refer_path[i,1] - self.refer_path[i-1,1]
                ds = np.sqrt(dx_s**2+dy_s**2)
                
            self.refer_path[i,2]=math.atan2(dy,dx) # yaw
            self.refer_path[i,3]=(ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2)) 
            self.refer_path[i,4] = ds
            
            for j in range(i+1):
                self.refer_path[i,5] += self.refer_path[j,4]
    
    def __call__(self,s):
        index = self.get_nearst_s(s)
        x_d =  self.refer_path[index,0]
        y_d =  self.refer_path[index,1]
        return x_d, y_d

    def get_nearst_s(self,s):
        index_list = np.where(abs(self.refer_path[:,5] - s) < 0.015)[0]
        distance = np.zeros(len(index_list))
        for i in range(len(index_list)):
            distance[i] = (s - self.refer_path[i,5])**2
        index = np.argmin(distance)
        first_index = index_list[0]
        index_d = first_index + index
        return index_d
            
    def get_k(self,s):
        index_k = self.get_nearst_s(s)
        k = self.refer_path[index_k,3]
        return k
    
    def get_len(self):
        return self.refer_path[len(self.refer_path)-1,5]
    
    def get_theta_r(self,s):
        index_theta_r =   self.get_nearst_s(s)
        theta_r = self.refer_path[index_theta_r,2]
        return theta_r
    
    def get_samples(self):
        return self.refer_path[:,5]
    
    def get_xy_list(self):
        return self.refer_path[:,0], self.refer_path[:,1]

refer_path = MyReferencePath()
path_x_list,path_y_list = refer_path.get_xy_list()
path_samples = refer_path.get_samples()
# plt.plot(refer_path.refer_path[:,5],refer_path.refer_path[:,3])
# plt.xlabel("s(m)")
# plt.ylabel("k(curvature)")
# plt.show()
