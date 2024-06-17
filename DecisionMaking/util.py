import sys
sys.path.append("C:\\Users\\sym02\\Desktop\\Research\\Extension\\codes\\decision_improve") 
import numpy as np
from Path.path import *
# from Path.path_ngsim import *
from Prediction.surrounding_prediction import *


class LeaderFollower_Uitl:
    '''
    formulate_group is a function to provide target groups for GSD
    get_all_constraint is a function to give all constraints every step
    
    '''
    def __init__(self,rho,a_max_acc_lon,a_max_brake_lon,a_min_brake_lon,vehicle_width,
                l,l_diag,mu,T,dt,lane_width,Th,d0,vehicle_num):
        self.rho = rho
        self.a_max_acc_lon = a_max_acc_lon
        self.a_max_brake_lon = a_max_brake_lon
        self.a_min_brake_lon = a_min_brake_lon
        self.vehicle_width = vehicle_width
        self.l = l
        self.l_diag = l_diag
        self.mu = mu
        self.T = T
        self.dt = dt
        self.lane_width = lane_width
        self.Th = Th
        self.d0 = d0
        self.vehicle_num = vehicle_num
        
        
    def set_decision_maker(self,GSD_):
        self.decision_maker = GSD_
        
    def get_onelane_lf(self,vehicle,xy_ego,forward_vector,path,x_list,y_list,sample,se):
        
        xy_left = vehicle[:,3:5]
    
        
        projection_s = np.zeros(xy_left.shape[0])
        projection_ey = np.zeros(xy_left.shape[0])
        for i in range(xy_left.shape[0]):
            s_map,ey_map = self.coordinate_remapping(path,x_list,y_list,sample,xy_left[i])
           
            projection_s[i] = s_map
            projection_ey[i] = ey_map
            
        projection_s = projection_s - se
        
        # 找出大于0的元素组成的列表及其索引
        positive_indices = [i for i, x in enumerate(projection_s) if x > 0]

        # 找出小于0的元素组成的列表及其索引
        negative_indices = [i for i, x in enumerate(projection_s) if x < 0]

        # 大于0中最小的的index和第二小的index
        positive_indices_sorted = sorted(positive_indices, key=lambda i: projection_s[i])

        leader_index = positive_indices_sorted[0] if positive_indices_sorted else None
        leader2_index = positive_indices_sorted[1] if len(positive_indices_sorted) > 1 else None

        
        # 小于0中最大的的index
        negative_indices_sorted = sorted(negative_indices, key=lambda i: projection_s[i], reverse=True)
        follower_index = negative_indices_sorted[0] if negative_indices_sorted else None
        follower2_index = leader_index

        proj_l = projection_s[leader_index] if leader_index is not None else None
        proj_l2 = projection_s[leader2_index] if leader2_index is not None else None
        proj_f = projection_s[follower_index] if follower_index is not None else None
        proj_f2 = projection_s[follower2_index] if follower2_index is not None else None      

        return (leader_index,leader2_index,follower_index,follower2_index),\
               (proj_l,proj_l2,proj_f,proj_f2),(projection_s+se,projection_ey)

    def get_egolane_lf(self,vehicle_ego,se):

        s_centre,ey_centre = vehicle_ego[:,0],vehicle_ego[:,1]
        ds_centre =  s_centre - se

        positive_indices = np.where(ds_centre > 0)[0]
        positive_values = ds_centre[positive_indices]

        # 小于0的子数组及其索引
        negative_indices = np.where(ds_centre < 0)[0]
        negative_values = ds_centre[negative_indices]

        # 对大于0的值进行排序并尝试获取最小和第二小的索引
        sorted_pos_indices = positive_indices[np.argsort(positive_values)]
        leader_index = sorted_pos_indices[0] if len(sorted_pos_indices) > 0 else None
        leader2_index = sorted_pos_indices[1] if len(sorted_pos_indices) > 1 else None

        # 尝试获取小于0的最大值的索引
        follower_index = negative_indices[np.argmax(negative_values)] if len(negative_indices) > 0 else None
        follower2_index = leader_index
        
        proj_l = ds_centre[leader_index] if leader_index is not None else None
        proj_l2 = ds_centre[leader2_index] if leader2_index is not None else None
        proj_f = ds_centre[follower_index] if follower_index is not None else None
        proj_f2 = ds_centre[follower2_index] if follower2_index is not None else None    

        return (leader_index,leader2_index,follower_index,follower2_index),\
               (proj_l,proj_l2,proj_f,proj_f2),(s_centre,ey_centre)
    

    def get_alllane_lf(self,path_ego,X0_g,path_now,vehicle_left,vehicle_centre,vehicle_right):

        '''
        X0 = [vx,vy,psi,s,ey,epsi]
        vehicle_xxx = np.array([s,ey,epsi,x,y,psi],
                            [.....])
        '''

        path, x_list, y_list, sample = get_path_info(path_now)
        se,ey_e = self.coordinate_remapping(path,x_list,y_list,sample,X0_g[0:2])
        theta = path.get_theta_r(se)
        forward_vector = [np.cos(theta),np.sin(theta)]
        xy_ego = [X0_g[0], X0_g[1]]
        
        if path_now == 0:
            lf_ego,proj_ego,proje_sey = self.get_egolane_lf(vehicle_left,se)
            lf_right,proj_right,projr_sey = self.get_onelane_lf(vehicle_centre,xy_ego,forward_vector,path,x_list,y_list,sample,se)    
            lf_most_right,proj_most_right,projmr_sey = self.get_onelane_lf(vehicle_right,xy_ego,forward_vector,path,x_list,y_list,sample,se)
            return lf_ego, lf_right, lf_most_right, proj_ego, proj_right, proj_most_right,proje_sey,projr_sey,projmr_sey
    
        elif path_now == 1:
            lf_ego,proj_ego, proje_sey = self.get_egolane_lf(vehicle_centre,se)
            lf_left, proj_left, projl_sey = self.get_onelane_lf(vehicle_left,xy_ego,forward_vector,path,x_list,y_list,sample,se) 
            lf_right, proj_right, projr_sey = self.get_onelane_lf(vehicle_right,xy_ego,forward_vector,path,x_list,y_list,sample,se) 
            return lf_ego, lf_left, lf_right, proj_ego, proj_left, proj_right, proje_sey, projl_sey, projr_sey
        
        elif path_now == 2:
            lf_ego,proj_ego, proje_sey = self.get_egolane_lf(vehicle_right,se)
            lf_left, proj_left, projl_sey = self.get_onelane_lf(vehicle_centre,xy_ego,forward_vector,path,x_list,y_list,sample,se)
            lf_most_left, proj_most_left, projml_sey = self.get_onelane_lf(vehicle_left,xy_ego,forward_vector,path,x_list,y_list,sample,se)     
            return lf_ego, lf_left, lf_most_left,proj_ego, proj_left,proj_most_left, proje_sey, projl_sey, projml_sey

    def lf_helper(self,group,l_index,f_index,vehicle,proj_sey):
        projection_s, projection_ey = proj_sey
        if l_index is not None:
            sl0,eyl0 = projection_s[l_index], projection_ey[l_index]
            epsil0,vl0,al0 = vehicle[l_index,2],vehicle[l_index,6],vehicle[l_index,7]
            xl0,yl0 = vehicle[l_index,3],vehicle[l_index,4]
            #只要拿到第一个得是对的，后面得就都是对的
            prediction_l = surrounding_vehicle_prediction(sl0,eyl0,epsil0,vl0,al0,self.dt,self.T)
            group['sl'] = prediction_l[0,:]
            group['eyl'] = prediction_l[1,:]
            group['vl'] = prediction_l[3,:]
            group['xl'] = xl0
            group['yl'] = yl0
            group['al'] = al0
        else:
            group['sl'] = None
            group['eyl'] = None
            group['vl'] = None  
            group['xl'] = None
            group['yl'] = None
            group['al'] = None
        if f_index is not None:
            sf0,eyf0 = projection_s[f_index], projection_ey[f_index]
            epsif0,vf0,af0 = vehicle[f_index,2],vehicle[f_index,6],vehicle[f_index,7]
            xf0,yf0 = vehicle[f_index,3],vehicle[f_index,4]
            prediction_f = surrounding_vehicle_prediction(sf0,eyf0,epsif0,vf0,af0,self.dt,self.T)
            group['sf'] = prediction_f[0,:]
            group['eyf'] = prediction_f[1,:]
            group['vf'] = prediction_f[3,:]
            group['xf'] = xf0
            group['yf'] = yf0
            group['af'] = af0
        else:
            group['sf'] = None
            group['eyf'] = None
            group['vf'] = None
            group['xf'] = None
            group['yf'] = None
            group['af'] = None
        return group
    
    def group_formulate(self,lf,vehicle,proj_f,proj_f2,proj_sey,ego_traj=None):
        if ego_traj is not None:
            l_index,l2_index,f_index,f2_index = lf
            group, group1 = {},{}
            # group['se'] = ego_traj[3]
            group['ve'] = ego_traj[0]
            
            group = self.lf_helper(group,l_index,f_index,vehicle,proj_sey)
            group['f_index'] = f_index    
            if l2_index is None and l_index is None:
                group1 = self.lf_helper(group1,l2_index,f_index,vehicle,proj_sey)  
                group1['proj_f'] = proj_f 
                group1['f_index'] = f_index
            else:
                group1 = self.lf_helper(group1,l2_index,l_index,vehicle,proj_sey) 
                group1['proj_f'] = proj_f2 
                group1['f_index'] = l_index           
            return group, group1
        else:
            l_index,l2_index,f_index,f2_index = lf     
            group,group1 = {},{}
            group = self.lf_helper(group,l_index,f_index,vehicle,proj_sey)
            group['proj_f'] = proj_f
            group['f_index'] = f_index
            if l2_index is None and l_index is None:
                group1 = self.lf_helper(group1,l2_index,f_index,vehicle,proj_sey)  
                group1['proj_f'] = proj_f 
                group1['f_index'] = f_index
            else:
                group1 = self.lf_helper(group1,l2_index,l_index,vehicle,proj_sey) 
                group1['proj_f'] = proj_f2 
                group1['f_index'] = f2_index        
            return group, group1   


    def formulate_gap_group(self,path_now,ego_traj,all_info,vehicle_left,vehicle_centre,vehicle_right):
        '''
        vehicle_xxx = np.array([s,ey,epsi,x,y,psi,vx,a],
                            [.....])
        '''    

        if path_now == 0:
            lf_ego, lf_right, lf_most_right, proj_ego, proj_right, proj_most_right,proje_sey,projr_sey,projmr_sey = all_info
            _,_,proj_f_ego,proj_f2_ego_forward = proj_ego
            _,_,proj_f_right,proj_f2_right = proj_right
            _,_,proj_f_most_right,proj_f2_most_right = proj_most_right
            
            vehicle_ego = vehicle_left
            ego_group,ego_forward_group = self.group_formulate(lf_ego,vehicle_ego,proj_f_ego,proj_f2_ego_forward,proje_sey,ego_traj)
            right_group1,right_group2 = self.group_formulate(lf_right,vehicle_centre,proj_f_right,proj_f2_right,projr_sey,ego_traj=None)
            most_right_group1,most_right_group2 = self.group_formulate(lf_most_right,vehicle_right,proj_f_most_right,proj_f2_most_right,projmr_sey,ego_traj=None)
            
            ego_group['name'],ego_forward_group['name'] = "L1","L2"
            right_group1['name'],right_group2['name'] = "C1","C2"
            most_right_group1['name'],most_right_group2['name'] = "R1","R2"

            L_groups = [ego_group,ego_forward_group]
            C_groups = [right_group1,right_group2]
            R_groups = [most_right_group1,most_right_group2]
            
            group_dict = {"L1":ego_group,"L2":ego_forward_group,"C1":right_group1,"C2":right_group2,"R1":most_right_group1,"R2":most_right_group2}
            return group_dict,ego_group
        
        elif path_now == 1:
            lf_ego, lf_left, lf_right, proj_ego, proj_left, proj_right, proje_sey, projl_sey, projr_sey = all_info
            _,_,proj_f_ego,proj_f2_ego_forward = proj_ego
            _,_,proj_f_left,proj_f2_left = proj_left
            _,_,proj_f_right,proj_f2_right = proj_right
            vehicle_ego = vehicle_centre
            ego_group,ego_forward_group = self.group_formulate(lf_ego,vehicle_ego,proj_f_ego,proj_f2_ego_forward,proje_sey,ego_traj)
            right_group,right_group1 = self.group_formulate(lf_right,vehicle_right,proj_f_right,proj_f2_right,projr_sey,ego_traj=None)
            left_group,left_group1 = self.group_formulate(lf_left,vehicle_left,proj_f_left,proj_f2_left,projl_sey,ego_traj=None)
         
            ego_group['name'],ego_forward_group['name'] = "C1","C2"
            right_group['name'],right_group1['name'] = "R1","R2"
            left_group['name'],left_group1['name'] = "L1","L2"
                
            L_groups = [left_group,left_group1]
            C_groups = [ego_group,ego_forward_group]
            R_groups = [right_group,right_group1]
            
            
            group_dict = {"L1":left_group,"L2":left_group1,"C1":ego_group,"C2":ego_forward_group,"R1":right_group,"R2":right_group1}
            return group_dict,ego_group
        
        elif path_now == 2:
            lf_ego, lf_left, lf_most_left,proj_ego,proj_left,proj_most_left,proje_sey, projl_sey, projml_sey = all_info
            _,_,proj_f_ego,proj_f2_ego_forward = proj_ego
            _,_,proj_f_left,proj_f2_left = proj_left
            _,_,proj_f_most_left,proj_f2_most_left = proj_most_left

            vehicle_ego = vehicle_right
            ego_group,ego_forward_group = self.group_formulate(lf_ego,vehicle_ego,proj_f_ego,proj_f2_ego_forward,proje_sey,ego_traj)
            left_group,left_group1 = self.group_formulate(lf_left,vehicle_centre,proj_f_left,proj_f2_left,projl_sey,ego_traj=None)
            most_left_group,most_left_group1 = self.group_formulate(lf_most_left,vehicle_left,proj_f_most_left,proj_f2_most_left,projml_sey,ego_traj=None)
            
            ego_group['name'],ego_forward_group['name'] = "R1","R2"
            left_group['name'],left_group1['name'] = "C1","C2"
            most_left_group['name'],most_left_group1['name'] = "L1","L2"
            
            L_groups = [most_left_group,most_left_group1]
            C_groups = [left_group,left_group1]
            R_groups = [ego_group,ego_forward_group]   
            
            group_dict = {"L1":most_left_group,"L2":most_left_group1,"C1":left_group,"C2":left_group1,"R1":ego_group,"R2":ego_forward_group}         
            return group_dict,ego_group
    
    def target_f_judge(self,proj_f_0):
        # it can be fixed later
        if proj_f_0 is not None and proj_f_0 <= -self.l/2:
            judger = True #说明可以开始dhocbf约束
        else:
            judger = False
        return judger
    
    def get_targetf_constraint(self,target_group,C_label):
        #这里有问题
        proj_f_0 = target_group['proj_f']
        judger = self.target_f_judge(proj_f_0)

        if judger:
            if C_label == "R":
                prediction_ey = target_group['eyf']  + self.lane_width
                # d_min_T =  prediction_ey + self.mu + self.vehicle_width
                d_min_T = prediction_ey + 3.5 
 
            elif C_label == "L":
                prediction_ey = target_group['eyf'] - self.lane_width
                # d_min_T =  prediction_ey - self.mu  - self.vehicle_width
                d_min_T = prediction_ey - 3.5 
        else:
            d_min_T = None
        return d_min_T 
    
    def get_lateral_dmin(self,index_len,constraint_index,vehicle,direction_signal,C_label):
        '''
        direction_signal: to give signal about whether the constraint is in left lane or right lane
        needs to exclude the target dmin_f
        这个地方其实是直接从原来的道路映射到desired lane上,而不是从 path now
        '''
        if index_len != 0:
            for item in constraint_index:
                state = vehicle[item]
                prediction = surrounding_vehicle_prediction(state[0],state[1],state[2],state[6],state[7],self.dt,self.T)
                if direction_signal == "constraint_right":
                    if C_label == "K":
                        projection_surround = prediction[1,:] - self.lane_width 
                    elif C_label == "L":
                        projection_surround = prediction[1,:] - 2*self.lane_width
                    elif C_label == "R":
                        projection_surround = prediction[1,:] 
                    d_min_T = projection_surround + self.mu + self.vehicle_width
                elif direction_signal == "constraint_left":
                    if C_label == "K":
                        projection_surround = self.lane_width - prediction[1,:] 
                    elif C_label == "L":
                        projection_surround = prediction[1,:] 
                    elif C_label == "R":
                        projection_surround = 2*self.lane_width - prediction[1,:] 
                    d_min_T = projection_surround - self.mu - self.vehicle_width
        else: 
            d_min_T = None
        return d_min_T    
    
    def get_index(self,proj,lf,target_df_index,C_label,direction_signal,C_label_additive):
        '''
        proj:the projection value on the ego vehicle's direction
        lf:the index of the leader and follower in vehicle list
        constraint_index:the list of index in a lane that coressponding 
        vehicles needed to be constrainted
        '''
        constraint_index = []

        for i in range(len(proj)):
            if (direction_signal == "constraint_right" and C_label == "L") or (direction_signal == "constraint_left" and C_label == "R") or (C_label_additive == "Probe"):
                if proj[i] is not None:
                    if np.abs(proj[i]) <= 2*self.l_diag:
                        constraint_index.append(lf[i])
            else:
                if proj[i] is not None:
                    if np.abs(proj[i]) <= 2*self.l_diag and lf[i] != target_df_index:
                        constraint_index.append(lf[i])                
        return constraint_index
    
    def get_surrounding_constraints(self,all_info,path_now,vehicle_left,vehicle_centre,vehicle_right,target_df_index,C_label,C_label_additive):
        #here are some problems to be fixed
        if path_now == 0:
            _, lf_right, _, _, proj_right, _, _ ,_ ,_  = all_info
            # print("proj_right in get_surrounding_constraints=",proj_right)
            direction_signal = "constraint_right"
            constraintR_index = self.get_index(proj_right,lf_right,target_df_index,C_label,direction_signal,C_label_additive)
            

            dR_min_T =  self.get_lateral_dmin(len(constraintR_index),constraintR_index,vehicle_centre,direction_signal,C_label)
            return dR_min_T
        
        elif path_now == 1:
            _, lf_left, lf_right, _, proj_left, proj_right, _, _, _ = all_info
            # print("proj_right in get_surrounding_constraints=",proj_right)
            # print("proj_left in get_surrounding_constraints=",proj_left)
            direction_signal_L = "constraint_left"
            direction_signal_R = "constraint_right"
            constraintR_index = self.get_index(proj_right,lf_right,target_df_index,C_label,direction_signal_R,C_label_additive)
            constraintL_index = self.get_index(proj_left,lf_left,target_df_index,C_label,direction_signal_L,C_label_additive)
            print("constraintR_index=",constraintR_index)
            print("constraintL_index=",constraintL_index)
            dL_min_T =  self.get_lateral_dmin(len(constraintL_index),constraintL_index,vehicle_left,direction_signal_L,C_label)
            dR_min_T =  self.get_lateral_dmin(len(constraintR_index),constraintR_index,vehicle_right,direction_signal_R,C_label)
            return dL_min_T, dR_min_T
        
        elif path_now == 2:
            _, lf_left, _, _,proj_left, _, _, _, _= all_info
            # print("proj_left in get_surrounding_constraints=",proj_left)
            direction_signal_L = "constraint_left"
            constraintL_index = self.get_index(proj_left,lf_left,target_df_index,C_label,direction_signal_L,C_label_additive)
            

            dL_min_T =  self.get_lateral_dmin(len(constraintL_index),constraintL_index,vehicle_centre,direction_signal_L,C_label)
            return dL_min_T
        
    def get_longitudinal_constraints(self,ve,prediction_vl_ego):
        second_paraml = np.zeros(self.T+1)
        third_paraml = np.zeros(self.T+1)
        vei = ve[0]
        for i in range(self.T+1):
            # vl = prediction_vl_ego[i]
            second_paraml[i] =  self.Th
            third_paraml[i] = self.d0
        return second_paraml, third_paraml
    
    def inquire_C_state(self,C_label, target_group):
        # print("target_group's proj_f=",target_group)
        if C_label != "K":
            projd_follower = target_group["proj_f"]
            follower_judge = self.target_f_judge(projd_follower)
        if C_label == "K":
            C_label_additive = "No Probe"
        elif C_label != "K" and not follower_judge:
            C_label_additive = "Probe"
        elif C_label != "K" and follower_judge:
            C_label_additive = "constraint"
        return C_label_additive
    
    def get_all_constraint(self,C_label,path_now,path_d,ego_group,path_ego,X0,X0_g,target_group,vehicle_left,vehicle_centre,vehicle_right,path_dindex,C_label_virtual):
        all_info = self.get_alllane_lf(path_ego,X0_g,path_now,vehicle_left,vehicle_centre,vehicle_right)
        
        ve = ego_group['ve']
        
        C_label_additive = self.inquire_C_state(C_label,target_group)
        
        x0_g_le = [ego_group['xl'],ego_group['yl']]
        x0_g_fe = [ego_group['xf'],ego_group['yf']]
        x0_g_lt = [target_group['xl'],target_group['yl']]
        x0_g_ft = [target_group['xf'],target_group['yf']]
        
        #本车道前车
        if ego_group['sl'] is not None:
            prediction_sl_ego = ego_group['sl'][0:self.T+1]
            prediction_vl_ego = ego_group['vl']
        else:
            prediction_sl_ego = None
            prediction_vl_ego = None
        
        #本车道后车
        if ego_group['sf'] is not None:
            prediction_sf_ego = ego_group['sf'][0:self.T+1]
            prediction_vf_ego = ego_group['vf']
        else:
            prediction_sf_ego = None
            prediction_vf_ego = None            
        
        # target group 前车
        if target_group["sl"] is not None:
            prediction_sl_target = target_group["sl"][0:self.T+1]
            prediction_vl_target = target_group['vl']
        else:
            prediction_sl_target = None
            prediction_vl_target = None
            
        target_df_index = target_group["f_index"]

        surround_constraints = self.get_surrounding_constraints(all_info,path_now,vehicle_left,vehicle_centre,vehicle_right,target_df_index,C_label_virtual,C_label_additive)
        if C_label_additive == "No Probe":
            second_param_l, third_param_l = self.get_longitudinal_constraints(ve,prediction_vl_ego)
            prediction_sl_ego = self.get_remap_vehicles(x0_g_le,prediction_vl_ego,path_dindex,path_d)
            prediction_sf_ego = self.get_remap_vehicles(x0_g_fe,prediction_vf_ego,path_dindex,path_d)
            if prediction_sl_ego is None:
                return prediction_sl_ego, prediction_sf_ego[0,:], second_param_l, third_param_l, surround_constraints
            elif prediction_sl_ego is not None:
                print("prediction_sl_ego=",type(prediction_sl_ego))
                print("prediction_sf_ego=",type(prediction_sf_ego))
                return prediction_sl_ego[0,:], prediction_sf_ego[0,:], second_param_l, third_param_l, surround_constraints
        
        elif C_label_additive == "Probe":#这个时候只有ahead的dhocbf
            second_param_tl, third_param_tl = self.get_longitudinal_constraints(ve,prediction_sl_target)
            prediction_ahead = self.get_remap_vehicles(x0_g_le,prediction_vl_ego,path_dindex,path_d)

            prediction_sf_ego = self.get_remap_vehicles(x0_g_fe,prediction_vf_ego,path_dindex,path_d)
            if prediction_sf_ego is None:
                return prediction_ahead, prediction_sf_ego, second_param_tl, third_param_tl, surround_constraints
            else:
                return prediction_ahead, prediction_sf_ego[0,:], second_param_tl, third_param_tl, surround_constraints
            
        elif C_label_additive == "constraint":
            second_param_tl, third_param_tl = self.get_longitudinal_constraints(ve,prediction_sl_target)
            print("prediction_vf_ego=",type(prediction_vf_ego))
            prediction_rear = self.get_remap_vehicles(x0_g_ft,prediction_vf_ego,path_dindex,path_d) # target grup的follower
            prediction_ahead = self.get_remap_vehicles(x0_g_le,prediction_vl_ego,path_dindex,path_d)
            prediction_sf_ego = self.get_remap_vehicles(x0_g_fe,prediction_vf_ego,path_dindex,path_d)
            prediction_sl_target = self.get_remap_vehicles(x0_g_lt,prediction_vl_target,path_dindex,path_d)
            if prediction_sl_target is None:
                return prediction_ahead, prediction_rear, prediction_sf_ego[0,:], prediction_sl_target, second_param_tl, third_param_tl, surround_constraints
            else:
                return prediction_ahead, prediction_rear, prediction_sf_ego[0,:], prediction_sl_target[0,:], second_param_tl, third_param_tl, surround_constraints           
    
    def coordinate_remapping(self,path_d,x_list,y_list,sample,x0_g_v):
        xy_stack = np.transpose(np.array([x_list,y_list])) - x0_g_v
        d = np.linalg.norm(xy_stack,ord=2, axis=1)
        min_index = np.argmin(d)
        s_map = sample[min_index]
        ey_map = d[min_index]
        
        theta_r = path_d.get_theta_r(s_map)
        sign = (x0_g_v[1]-y_list[min_index])*np.cos(theta_r) - (x0_g_v[0]-x_list[min_index])*np.sin(theta_r)
        if sign > 0:
            pass
        elif sign < 0:
            ey_map = -ey_map
        
        return s_map,ey_map
    
    def get_remap_vehicles(self,x0_g_l,prediction_vl_ego,path_dindex,path_d):
        if x0_g_l[0] is not None:
            prediction_ahead = np.zeros((2,self.T+1))
            _,x_list,y_list,sample = get_path_info(path_dindex)
            sl_ego_remap,eyl_ego_remap = self.coordinate_remapping(path_d,x_list,y_list,sample,x0_g_l)
            prediction_ahead[0,0],prediction_ahead[1,0] = sl_ego_remap, eyl_ego_remap
            for i in range(1,self.T+1):
                prediction_ahead[0,i] = prediction_vl_ego[i-1]*self.dt + prediction_ahead[0,i-1]
                prediction_ahead[1,i] = eyl_ego_remap
        else:
            prediction_ahead = None   
        return prediction_ahead
    
    
    def get_index_for_comparison(self,proj,lf):
        '''
        proj:the projection value on the ego vehicle's direction
        lf:the index of the leader and follower in vehicle list
        constraint_index:the list of index in a lane that coressponding 
        vehicles needed to be constrainted
        '''
        constraint_index = []
        for i in range(len(proj)):
            if proj[i] is not None:
                if np.abs(proj[i]) <= 2*self.l_diag:
                    constraint_index.append(lf[i])                
        return constraint_index
    
    def get_surrounding_constraints_for_comparison(self,all_info,path_now,vehicle_left,vehicle_centre,vehicle_right,C_label):
        #here are some problems to be fixed
        if path_now == 0:
            _, lf_right, _, _, proj_right, _, _ ,_ ,_  = all_info
            # print("proj_right in get_surrounding_constraints=",proj_right)
            direction_signal = "constraint_right"
            constraintR_index = self.get_index_for_comparison(proj_right,lf_right)
            
            dR_min_T =  self.get_lateral_dmin(len(constraintR_index),constraintR_index,vehicle_centre,direction_signal,C_label)
            return dR_min_T
        
        elif path_now == 1:
            _, lf_left, lf_right, _, proj_left, proj_right, _, _, _ = all_info
            # print("proj_right in get_surrounding_constraints=",proj_right)
            # print("proj_left in get_surrounding_constraints=",proj_left)
            direction_signal_L = "constraint_left"
            direction_signal_R = "constraint_right"
            constraintR_index = self.get_index_for_comparison(proj_right,lf_right)
            constraintL_index = self.get_index_for_comparison(proj_left,lf_left)
            dL_min_T =  self.get_lateral_dmin(len(constraintL_index),constraintL_index,vehicle_left,direction_signal_L,C_label)
            dR_min_T =  self.get_lateral_dmin(len(constraintR_index),constraintR_index,vehicle_right,direction_signal_R,C_label)
            return dL_min_T, dR_min_T
        
        elif path_now == 2:
            _, lf_left, _, _,proj_left, _, _, _, _= all_info
            # print("proj_left in get_surrounding_constraints=",proj_left)
            direction_signal_L = "constraint_left"
            constraintL_index = self.get_index_for_comparison(proj_left,lf_left)
            dL_min_T =  self.get_lateral_dmin(len(constraintL_index),constraintL_index,vehicle_centre,direction_signal_L,C_label)
            return dL_min_T
                                          
    def get_all_constraint_for_comparison(self,path_now,path_d,ego_group,path_ego,X0_g,target_group,vehicle_left,vehicle_centre,vehicle_right,path_dindex,C_label):

        all_info = self.get_alllane_lf(path_ego,X0_g,path_now,vehicle_left,vehicle_centre,vehicle_right)
        
        ve = ego_group['ve']
        
        x0_g_le = [ego_group['xl'],ego_group['yl']]
        x0_g_fe = [ego_group['xf'],ego_group['yf']]
        
        #本车道前车
        if ego_group['sl'] is not None:
            prediction_sl_ego = ego_group['sl'][0:self.T+1]
            prediction_vl_ego = ego_group['vl']
        else:
            prediction_sl_ego = None
            prediction_vl_ego = None
        
        #本车道后车
        if ego_group['sf'] is not None:
            prediction_sf_ego = ego_group['sf'][0:self.T+1]
            prediction_vf_ego = ego_group['vf']
        else:
            prediction_sf_ego = None
            prediction_vf_ego = None            

        surround_constraints = self.get_surrounding_constraints_for_comparison(all_info,path_now,vehicle_left,vehicle_centre,vehicle_right,C_label)
        second_param_l, third_param_l = self.get_longitudinal_constraints(ve,prediction_vl_ego)
        prediction_sl_ego = self.get_remap_vehicles(x0_g_le,prediction_vl_ego,path_dindex,path_d)
        prediction_sf_ego = self.get_remap_vehicles(x0_g_fe,prediction_vf_ego,path_dindex,path_d)
        if prediction_sl_ego is None:
            return prediction_sl_ego, prediction_sf_ego[0,:], second_param_l, third_param_l, surround_constraints
        elif prediction_sl_ego is not None:
            return prediction_sl_ego[0,:], prediction_sf_ego[0,:], second_param_l, third_param_l, surround_constraints
        
        
        
        
    def inquire_C_state_for_noadapt(self,C_label, target_group):
        # print("target_group's proj_f=",target_group)
        if C_label != "K":
            projd_follower = target_group["proj_f"]
            follower_judge = self.target_f_judge(projd_follower)
        if C_label == "K":
            C_label_additive = "No Probe"
        elif C_label != "K" and not follower_judge:
            C_label_additive = "No Probe"
        elif C_label != "K" and follower_judge:
            C_label_additive = "constraint"
        return C_label_additive
    
    def get_all_constraint_for_noadapt(self,C_label,path_now,path_d,ego_group,path_ego,X0,X0_g,target_group,vehicle_left,vehicle_centre,vehicle_right,path_dindex,C_label_virtual):
        all_info = self.get_alllane_lf(path_ego,X0_g,path_now,vehicle_left,vehicle_centre,vehicle_right)
        
        ve = ego_group['ve']
        
        C_label_additive = self.inquire_C_state_for_noadapt(C_label,target_group)
        
        x0_g_le = [ego_group['xl'],ego_group['yl']]
        x0_g_fe = [ego_group['xf'],ego_group['yf']]
        x0_g_lt = [target_group['xl'],target_group['yl']]
        x0_g_ft = [target_group['xf'],target_group['yf']]
        
        #本车道前车
        if ego_group['sl'] is not None:
            prediction_sl_ego = ego_group['sl'][0:self.T+1]
            prediction_vl_ego = ego_group['vl']
        else:
            prediction_sl_ego = None
            prediction_vl_ego = None
        
        #本车道后车
        if ego_group['sf'] is not None:
            prediction_sf_ego = ego_group['sf'][0:self.T+1]
            prediction_vf_ego = ego_group['vf']
        else:
            prediction_sf_ego = None
            prediction_vf_ego = None            
        
        # target group 前车
        if target_group["sl"] is not None:
            prediction_sl_target = target_group["sl"][0:self.T+1]
            prediction_vl_target = target_group['vl']
        else:
            prediction_sl_target = None
            prediction_vl_target = None
            
        target_df_index = target_group["f_index"]

        surround_constraints = self.get_surrounding_constraints(all_info,path_now,vehicle_left,vehicle_centre,vehicle_right,target_df_index,C_label_virtual,C_label_additive)
        if C_label_additive == "No Probe":
            second_param_l, third_param_l = self.get_longitudinal_constraints(ve,prediction_vl_ego)
            prediction_sl_ego = self.get_remap_vehicles(x0_g_le,prediction_vl_ego,path_dindex,path_d)
            prediction_sf_ego = self.get_remap_vehicles(x0_g_fe,prediction_vf_ego,path_dindex,path_d)
            if prediction_sl_ego is None:
                return prediction_sl_ego, prediction_sf_ego[0,:], second_param_l, third_param_l, surround_constraints
            elif prediction_sl_ego is not None:
                print("prediction_sl_ego=",type(prediction_sl_ego))
                print("prediction_sf_ego=",type(prediction_sf_ego))
                return prediction_sl_ego[0,:], prediction_sf_ego[0,:], second_param_l, third_param_l, surround_constraints
        
        elif C_label_additive == "Probe":#这个时候只有ahead的dhocbf
            second_param_tl, third_param_tl = self.get_longitudinal_constraints(ve,prediction_sl_target)
            prediction_ahead = self.get_remap_vehicles(x0_g_le,prediction_vl_ego,path_dindex,path_d)

            prediction_sf_ego = self.get_remap_vehicles(x0_g_fe,prediction_vf_ego,path_dindex,path_d)
            if prediction_sf_ego is None:
                return prediction_ahead, prediction_sf_ego, second_param_tl, third_param_tl, surround_constraints
            else:
                return prediction_ahead, prediction_sf_ego[0,:], second_param_tl, third_param_tl, surround_constraints
            
        elif C_label_additive == "constraint":
            second_param_tl, third_param_tl = self.get_longitudinal_constraints(ve,prediction_sl_target)
            print("prediction_vf_ego=",type(prediction_vf_ego))
            prediction_rear = self.get_remap_vehicles(x0_g_ft,prediction_vf_ego,path_dindex,path_d) # target grup的follower
            prediction_ahead = self.get_remap_vehicles(x0_g_le,prediction_vl_ego,path_dindex,path_d)
            prediction_sf_ego = self.get_remap_vehicles(x0_g_fe,prediction_vf_ego,path_dindex,path_d)
            prediction_sl_target = self.get_remap_vehicles(x0_g_lt,prediction_vl_target,path_dindex,path_d)
            if prediction_sl_target is None:
                return prediction_ahead, prediction_rear, prediction_sf_ego[0,:], prediction_sl_target, second_param_tl, third_param_tl, surround_constraints
            else:
                return prediction_ahead, prediction_rear, prediction_sf_ego[0,:], prediction_sl_target[0,:], second_param_tl, third_param_tl, surround_constraints    