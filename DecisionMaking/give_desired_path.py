import numpy as np
import sys
sys.path.append("C:\\Users\\sym02\\Desktop\\Research\\Extension\\codes\\decision_improve") 
from Path.path import *
# from Path.path_ngsim import *
import Control.utils as util

def get_nearst_xy(xB,yB,x0_g):
    xy_stack = np.transpose(np.array([xB,yB])) - x0_g
    d = np.linalg.norm(xy_stack,ord=2, axis=1)
    min_index = np.argmin(d)
    d_min = d[min_index]
    x,y = xB[min_index], yB[min_index]
    return x, y, d_min, min_index

def get_sign(pathS,index,pathB,x_ego,x,y_ego,y):
    s = pathS[index]
    theta_r = pathB.get_theta_r(s)
    sign = (y_ego - y)*np.cos(theta_r) - (x_ego - x)*np.sin(theta_r)
    return sign

def judge_current_position(x0_g,x_bound,y_bound,path_bound,path_bound_sample):
    x_ego, y_ego = x0_g[0], x0_g[1]
    xB_left, xB_right = x_bound[0], x_bound[1]
    yB_left, yB_right = y_bound[0], y_bound[1]
    pathB_left, pathB_right = path_bound[0], path_bound[1]
    pathS_left, pathS_right = path_bound_sample[0], path_bound_sample[1]
    
    x_left,y_left,d_left,index_left = get_nearst_xy(xB_left, yB_left,x0_g)
    x_right,y_right,d_right,index_right = get_nearst_xy(xB_right, yB_right,x0_g)

    if d_left < d_right:
        sign = get_sign(pathS_left,index_left,pathB_left,x_ego,x_left,y_ego,y_left)
        if sign > 0:
            path_now = 0
        elif sign < 0:
            path_now = 1
    elif d_right < d_left:
        sign = get_sign(pathS_right,index_right,pathB_right,x_ego,x_right,y_ego,y_right)
        if sign > 0:
            path_now = 1
        elif sign < 0:
            path_now = 2
    elif d_right == d_left:
        path_now = 1
    return path_now

def give_desired_path(desired_group,path_now):
    print("path_now in give desired path=",path_now)
    print("target_lane in give desired path=",desired_group['name'])
    target_lane = desired_group['name']

    if target_lane == "C1" or target_lane == "C2":
        path_d = path2c

    elif target_lane == "L1" or target_lane == "L2":
        path_d = path1c

    elif target_lane == "R1" or target_lane == "R2":
        path_d = path3c
   
    return path_d


def repropagate(path_d,sample,x_list,y_list,x0_g,x0):
    
    s, ey, epsi = util.find_frenet_coord(path_d,x_list,y_list,sample,x0_g)
    x_next = x0
    x_next[3] = s
    x_next[4] = ey
    x_next[5] = epsi      

    return x_next

def post_process(x0,desired_group):
    # 这里比较一下 desired_group的前车与 ego vehicle 的距离
    target_ahead = desired_group['sl'][0]
    ego_s = x0[3]
    print("target_ahead=",target_ahead)
    print("ego_s=",ego_s)
    dis = abs(target_ahead - ego_s)
    is_short = False
    if dis <= 6.0:
        is_short = True
    return is_short


def Decision_info(x0,x0_g,path_center_list,sample_center,x_center,y_center,bound,desired_group,path_now,path_nowindex):
    #m貌似是这个地方的问题
    path_d = give_desired_path(desired_group,path_now)
    path_dindex = np.where(path_center_list==path_d)[0][0]
    sample,x_list,y_list = sample_center[path_dindex],x_center[path_dindex],y_center[path_dindex]
    # is_short = post_process(x0,desired_group)
    if path_now != path_d:
        if path_dindex > path_nowindex:
            C_label = "R"
        else:
            C_label = "L"
        x0_update = repropagate(path_d,sample,x_list,y_list,x0_g,x0)
        print("path_d=",path_d)
        print("path_dindex=",path_dindex)
        return path_d, path_dindex,C_label, sample,x_list,y_list,x0_update
    
    else:
        x0_update = repropagate(path_d,sample,x_list,y_list,x0_g,x0)
        C_label = "K"
        return path_d, path_dindex,C_label, sample, x_list, y_list,x0_update
    
    
    
    

