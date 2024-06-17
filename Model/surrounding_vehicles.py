import sys
sys.path.append(r"C:\Users\sym02\Desktop\Research\Extension\codes\copy_test") 
import numpy as np
import copy
from Model.Surrounding_model import *
from Model.surrounding_params import *
import random
import pickle
from Path.path import *
from Control.utils import find_frenet_coord, plot_car, create_rectangle

param = surrounding_params()

class Surrounding_Vehicles:
    def __init__(self,steer_range,dt,bound,file_dir=None):
        self.file_dir = file_dir
        self.left_vehicle_all = self.vehicle_initialization()
        self.center_vehicle_all = self.vehicle_initialization()
        self.right_vehicle_all = self.vehicle_initialization()
                
        if self.file_dir is None:   
            self.vd_left_all = self.vd_set() 
            self.vd_center_all = self.vd_set() 
            self.vd_right_all = self.vd_set()
            self.vehicle_left = self.x0_set(path1c)
            self.vehicle_center = self.x0_set(path2c)
            self.vehicle_right = self.x0_set(path3c)
        else:
            params = self.params_from_pickle()
            self.vd_left_all = params[1][0]
            self.vd_center_all = params[1][1]
            self.vd_right_all = params[1][2]
            self.vehicle_left = params[0][0]
            self.vehicle_center = params[0][1]
            self.vehicle_right = params[0][2]

                        
        self.left_initial_set = copy.deepcopy(self.vehicle_left)
        self.center_initial_set = copy.deepcopy(self.vehicle_center)
        self.right_initial_set = copy.deepcopy(self.vehicle_right)
        
        self.singleLane_num = len(self.left_vehicle_all)
        self.steer_range = steer_range
        
        self.dt = dt
        self.bound = bound
        
    def vehicle_initialization(self):
        return [Curved_Road_Vehicle(**param), Curved_Road_Vehicle(**param), Curved_Road_Vehicle(**param), Curved_Road_Vehicle(**param),Curved_Road_Vehicle(**param)]
            
    def vd_set(self):
        return [random.uniform(9,15), random.uniform(9,15), random.uniform(9,15), random.uniform(9,15), random.uniform(9,15)]

    def x0_set(self,path):
        s1,s2,s3,s4,s5 = random.uniform(0,10), random.uniform(50,60), random.uniform(80,100),random.uniform(120,140),random.uniform(160,180)
        vx1,vx2,vx3,vx4,vx5 = random.uniform(7.0,12.0), random.uniform(7.0,12.0), random.uniform(7.0,12.0), random.uniform(7.0,12.0), random.uniform(7.0,12.0)
        x1,x2,x3,x4,x5 = path(s1)[0], path(s2)[0], path(s3)[0], path(s4)[0], path(s5)[0]
        y1,y2,y3,y4,y5 = path(s1)[1], path(s2)[1], path(s3)[1], path(s4)[1], path(s5)[1]
        psi1,psi2,psi3,psi4,psi5 = path.get_theta_r(s1),path.get_theta_r(s2),path.get_theta_r(s3),path.get_theta_r(s4),path.get_theta_r(s5)
        return np.array([[s1,0,0,x1,y1,psi1,vx1,0], 
                         [s2,0,0,x2,y2,psi2,vx2,0], 
                         [s3,0,0,x3,y3,psi3,vx3,0], 
                         [s4,0,0,x4,y4,psi4,vx4,0], 
                         [s5,0,0,x5,y5,psi5,vx5,0]])
        
    def params_from_pickle(self):
        file_content = open(self.file_dir,'rb')
        content = pickle.load(file_content)
        pos_params = content['initial_params']
        vd_params = content['initial_vds']
        vehicle_left, vehicle_center, vehicle_right = pos_params[0], pos_params[1], pos_params[2]
        vd_left, vd_center, vd_right = vd_params[0], vd_params[1], vd_params[2]
        return (vehicle_left, vehicle_center, vehicle_right),(vd_left, vd_center, vd_right)
    
    def update_vehicle_states(self,vehicle_all,vehicle,vd_,path,samples,xc,yc):
        for i in range(self.singleLane_num):
            if i == (self.singleLane_num-1):
                s_ahead = None
                v_ahead = None
            else:
                s_ahead = vehicle[i+1][0]
                v_ahead = vehicle[i+1][6]
            s = vehicle[i][0]
            v = vehicle[i][6]
            vd = vd_[i]
            x_next, y_next, psi_next, v_next, _, a = vehicle_all[i].update_states(s, v, vd, s_ahead, v_ahead, path, self.steer_range)
            x0_g = [x_next, y_next, psi_next]
            s_next, ey_next, epsi_next = find_frenet_coord(path,xc,yc,samples,x0_g)
            vehicle[i] = [s_next, ey_next, epsi_next, x_next, y_next, psi_next, v_next, a]
    
    def total_update(self):
        self.update_vehicle_states(self.left_vehicle_all,self.vehicle_left,self.vd_left_all,path1c,samples1c,x1c,y1c)
        self.update_vehicle_states(self.center_vehicle_all,self.vehicle_center,self.vd_center_all,path2c,samples2c,x2c,y2c)
        self.update_vehicle_states(self.right_vehicle_all,self.vehicle_right,self.vd_right_all,path3c,samples3c,x3c,y3c)
    
    def get_vehicles_states(self):
        return self.vehicle_left, self.vehicle_center, self.vehicle_right
    
    def get_path_ego(self,path_now):
        if path_now == 0:
            path_ego = path1c
        elif path_now == 1:
            path_ego = path2c
        else:
            path_ego = path3c
        return path_ego
    
    def plot_vehicles(self,vehicle_length,vehicle_width,color):
        for i in range(self.singleLane_num):
            plot_car(self.vehicle_left[i][3],self.vehicle_left[i][4],self.vehicle_left[i][5],length=vehicle_length, width=vehicle_width,color=color)
            plt.text(self.vehicle_left[i][3]-1.3,self.vehicle_left[i][4]-0.3, "{} m/s".format(round(self.vehicle_left[i][6],1)),c='blue',fontsize=4,style='oblique')
            plt.text(self.vehicle_left[i][3]-1.3,self.vehicle_left[i][4]+0.6, "{} m".format(round(self.vehicle_left[i][0],1)),c='blue',fontsize=4,style='oblique')
            plot_car(self.vehicle_center[i][3],self.vehicle_center[i][4],self.vehicle_center[i][5],length=vehicle_length, width=vehicle_width,color=color)
            plt.text(self.vehicle_center[i][3]-1.3,self.vehicle_center[i][4]-0.3, "{} m/s".format(round(self.vehicle_center[i][6],1)),c='k',fontsize=4,style='oblique')
            plt.text(self.vehicle_center[i][3]-1.3,self.vehicle_center[i][4]+0.6, "{} m".format(round(self.vehicle_center[i][0],1)),c='k',fontsize=4,style='oblique')
            plot_car(self.vehicle_right[i][3],self.vehicle_right[i][4],self.vehicle_right[i][5],length=vehicle_length, width=vehicle_width,color=color)
            plt.text(self.vehicle_right[i][3]-1.3,self.vehicle_right[i][4]-0.3, "{} m/s".format(round(self.vehicle_right[i][6],1)),c='k',fontsize=4,style='oblique')
            plt.text(self.vehicle_right[i][3]-1.3,self.vehicle_right[i][4]+0.6, "{} m/s".format(round(self.vehicle_right[i][0],1)),c='k',fontsize=4,style='oblique')
            
    def get_rectangles(self,vehicles):
        rectangles = []
        for i in range(self.singleLane_num):
            rec = create_rectangle(vehicles[i][3],vehicles[i][4],3.5,1.2,vehicles[i][5])
            rectangles.append(rec)
        return rectangles
    
    def get_all_rectangles(self):
        left_rectangles = self.get_rectangles(self.vehicle_left)
        center_rectangles = self.get_rectangles(self.vehicle_center)
        right_rectangles = self.get_rectangles(self.vehicle_right)
        all_rectangles = left_rectangles + center_rectangles + right_rectangles
        return all_rectangles
    
    def S_obs_calc(self,ego_rect):
        all_rectangles = self.get_all_rectangles()
        
        S_obs_list = []
        for rect in all_rectangles:
            s_obs = ego_rect.distance(rect)
            S_obs_list.append(s_obs)
        S_obs_min = min(S_obs_list)
        
        return S_obs_min
            