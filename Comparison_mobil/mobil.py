import math
import numpy as np
from Path.path import *

class MOBIL:
    
    """Creating the DriverModel specific to the created vehicle based on
    the Intelligent Driver Model and the MOBIL lane change model
    """

    def __init__(self, v_0,s_0,a,b,b_safe,delta,T,politeness,change_threshold) :

        self.v_0 = v_0 # Max desired speed of vehicle
        self.s_0 = s_0 # Min desired distance between vehicles
        self.a = a # Max acceleration
        self.b = b # Comfortable deceleration
        self.b_safe = b_safe
        self.delta = delta # Acceleration component
        self.T = T #  Time safe headway
        self.politeness = politeness # Change lane politeness
        self.change_threshold = change_threshold # Change lane threshold


    def calc_acceleration(self, v, surrounding_v, s):

        """Calculates the vehicle acceleration based on the IDM

        Args:
            v (float): current vehicle velocity
            surrounding_v (float): velocity of other vehicle
            s (float): current actual distance

        Returns:
            float: vehicle acceleration
        """
        if surrounding_v is None:
            surrounding_v = 20.0
        delta_v = v - surrounding_v
        s_star = self.s_0 + max(0, self.T * v + (v * delta_v) / (2 * math.sqrt(self.a * self.b)))

        return (self.a * (1 - math.pow(v/self.v_0, self.delta) - math.pow(s_star/s, 2)))


    def calc_disadvantage(self, v, new_surrounding_v, new_surrounding_dist, old_surrounding_v, old_surrounding_dist):

        # Acceleration of the trailing vehicle behind the current vehicle
        new_back_acceleration = self.calc_acceleration(v, new_surrounding_v, new_surrounding_dist)

        # Acceleration of the currently investigated vehicle
        current_acceleration = self.calc_acceleration(v, old_surrounding_v, old_surrounding_dist)

        # Disadvantage on the trailing vehicle if lane changed
        disadvantage = current_acceleration - new_back_acceleration

        return disadvantage, new_back_acceleration


    def calc_incentive(self,  v, new_front_v, new_front_dist,
                    old_front_v, old_front_dist, disadvantage, new_back_accel):

        # IDM
        # Acceleration of the new front vehicle in the targeted lane
        new_front_acceleration = self.calc_acceleration(v=v, surrounding_v=new_front_v, s=new_front_dist)

        # Acceleration of the currently investigated vehicle
        current_acceleration = self.calc_acceleration(v=v, surrounding_v=old_front_v, s=old_front_dist)
    
        # MOBIL incentive equation
        change_incentive = new_front_acceleration - current_acceleration - (self.politeness * disadvantage) > self.change_threshold 

        # MOBIL kinematic-based safety criterion
        safety_criterion = new_back_accel >= -self.b_safe

        return change_incentive and safety_criterion, new_front_acceleration - current_acceleration
    
    def make_decision_oneside(self, s_e, v_e, vl_e, sl_e, group): 
        print("group=",group)   
        if group['sl'] is not None:
            sl= group['sl'][0]
            vl = group['vl'][0]
        else:
            sl= 3000.0
            vl = 100.0          

        if group['sf'] is not None:
            sf= group['sf'][0]
            vf = group['vf'][0]
            proj_f = group['proj_f']
        else:
            sf= 0.0
            vf = 0.0             
            proj_f = -100.0
        
        old_distance_lf = sl - sf
        
        if sl_e is None:
            sl_e = 3000.0
            sl_e = 100.0
        new_distance_le = sl - s_e
        old_distance_le = sl_e - s_e
        
        disadv_left, new_back_acc_left = self.calc_disadvantage(vf, v_e, abs(proj_f), vl, old_distance_lf)
        change_signal_left, change_incentive = self.calc_incentive(v_e, vl, new_distance_le, vl_e, old_distance_le, disadv_left, new_back_acc_left)        
        return change_signal_left,change_incentive
    
    def make_decision(self,group_dict,ego_group,x0,x0_g,path_now_index):
        ego_name = ego_group['name']
        path, xc, yc, samplesc = get_path_info(path_now_index)
        s_e, _ = coordinate_remapping(path,xc,yc,samplesc,x0_g[0:2])
        v_e = x0[0]  
        if ego_group['sl'] is not None:
            vl_e = ego_group['vl'][0] 
            sl_e = ego_group['sl'][0] 
        else:
            vl_e = ego_group['vl']
            sl_e = ego_group['sl']
         
        if ego_name == "C1":
            left_group = group_dict['L1']  #这些都是被映射在current lane上的
            right_group = group_dict['R1'] 
            left_change_signal,left_change_incentive = self.make_decision_oneside(s_e, v_e, vl_e, sl_e, left_group)
            right_change_signal,right_change_incentive = self.make_decision_oneside(s_e, v_e, vl_e, sl_e, right_group) 
            if left_change_signal == True and right_change_signal == False:
                desired_group = group_dict['L1']
            elif left_change_signal == False and right_change_signal == True:
                desired_group = group_dict['R1']
            elif left_change_signal == True and right_change_signal == True:
                if left_change_incentive > right_change_incentive:
                    desired_group = group_dict['L1']
                else:
                    desired_group = group_dict['R1']
            else:
                desired_group = group_dict['C1']
            return desired_group
        
        elif ego_name == "L1":
            right_group = group_dict['C1']
            right_change_signal,right_change_incentive = self.make_decision_oneside(s_e, v_e, vl_e, sl_e, right_group) 
            if right_change_signal:
                desired_group = group_dict['C1']
            else:
                desired_group = group_dict['L1']
            return desired_group
                       
        elif ego_name == "R1":
            left_group = group_dict['C1']
            left_change_signal,left_change_incentive = self.make_decision_oneside(s_e, v_e, vl_e, sl_e, left_group)        
            if left_change_signal:
                desired_group = group_dict['C1']
            else:
                desired_group = group_dict['R1']
            return desired_group

def get_path_info(path_dindex):
    if path_dindex == 0:
        return path1c, x1c, y1c, samples1c
    if path_dindex == 1:
        return path2c, x2c, y2c, samples2c
    if path_dindex == 2:
        return path3c, x3c, y3c, samples3c
    
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