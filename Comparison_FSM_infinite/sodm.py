import numpy as np

class sodm:
    def __init__(self,Bc,d0,Bc_l,alpha_l,rx,H_pred,dt,desired_v):
        self.Bc = Bc
        self.Bc_l = Bc_l
        self.d0 = d0
        self.alpha_l = alpha_l
        self.rx = rx
        self.H_pred = H_pred
        self.dt = dt
        self.desired_v = desired_v
        
    
    def Clac_threshold_F(self,v_l):
        th = 4*v_l**2/(3*np.sqrt(3)*self.Bc) + self.d0
        return th

    def Clac_threshold_L(self,v_l):
        th = 4*v_l**2/(3*np.sqrt(3)*self.Bc_l) + self.d0
        return th

    def threshold_demand_checking_l(self,X0):
        th_checking = (1 + self.alpha_l)*X0[0] + self.rx 
        return th_checking

    def threshold_demand_checking_f(self):
        th_checking = self.rx 
        return th_checking

    #现在有的是 6 个 group dict，要用的只有 L1，R1 以及 ego group
    #这个函数是计算每个车道上的leader与follower未来5个step之中的最小距离
    def Clac_X_d(self, vehicle_group, X0, lf_signal, a_ego):
        info = vehicle_group
        if info["s{}".format(lf_signal)] is not None:
            X,V,a = info["s{}".format(lf_signal)][0],info["v{}".format(lf_signal)][0],info["a{}".format(lf_signal)]  #先拿到前后车的信息
        else:
            X,V,a = 0,0,0
        X_list,V_list = np.array([X]),np.array([V])
        Ego_X_list,Ego_V_list = np.array([X0[3]]),np.array([X0[0]])
        for i in range(self.H_pred):
            L_Xmid = X_list[i] + V_list[i]*self.dt + self.dt**2/2*a
            L_Vmid = V_list[i] + a*self.dt
            X_list = np.append(X_list,L_Xmid)
            V_list = np.append(V_list,L_Vmid)
            if a_ego is None:
                a_ego = [0.0]
            Ego_Xmid = Ego_X_list[i] + Ego_V_list[i]*self.dt + self.dt**2/2*a_ego
            Ego_Vmid = Ego_V_list[i] + a_ego*self.dt
            Ego_X_list =np.append(Ego_X_list,Ego_Xmid)
            Ego_V_list = np.append(Ego_V_list,Ego_Vmid)
        X_diff_list = abs(X_list - Ego_X_list)
        X_diff_min = np.min(X_diff_list)
        return X_diff_min, V

        
    def Give_signal(self,a_ego,change_distance,group_dict, ego_group,path_now,X0):
        
        left_group,center_group,right_group = group_dict["L1"], group_dict["C1"],group_dict["R1"]
        th_checking_l = self.threshold_demand_checking_l(X0)
        th_checking_f = self.threshold_demand_checking_f()
        
        if path_now == 0:#这是在左车道
            Velocity_signal_L = 0
            Safe_signal_L = 0
            if left_group["sl"] is not None:#L1有前车
                if center_group["sl"] is not None:
                    #本车道的前车
                    X_diffLE_min, Le_V = self.Clac_X_d(center_group, X0, "l", a_ego)
                    #左车道的前车
                    X_diffLU_min, Lu_V = self.Clac_X_d(left_group, X0, "l",a_ego)
                    #本车道的后车
                    X_diffFE_min, Fe_V = self.Clac_X_d(center_group, X0, "f",a_ego)
                    #临界条件
                    threshold_F = self.Clac_threshold_F(Fe_V)
                    threshold_L = self.Clac_threshold_L(Le_V)
                    
                    Lu_V = left_group["vl"][0]
                    if X_diffLE_min >= threshold_L and X_diffLE_min >= th_checking_l and X_diffFE_min >= threshold_F and X_diffFE_min >= th_checking_f:
                        Safe_signal_R = 1
                    else:
                        Safe_signal_R = 0
                    if round(Lu_V,1) >= round(Le_V,1):
                        Velocity_signal_R = 0
                    else: 
                        Velocity_signal_R = 1        
                    if X_diffLE_min > change_distance:
                        Velocity_signal_R = 1    
                    if X_diffLU_min > change_distance:
                        Velocity_signal_R = 0
                        
                if center_group["sl"] is None:
                    Velocity_signal_R = 1
                    X_diffFE_min, Fe_V = self.Clac_X_d(center_group, X0, "f", a_ego)
                    threshold_F = self.Clac_threshold_F(Fe_V)
                    if X_diffFE_min >= threshold_F and X_diffFE_min >= th_checking_f: 
                        Safe_signal_R = 1
                    else:
                        Safe_signal_R = 0                  
            if left_group["sl"] is None:
                Safe_signal_R = 0
                Velocity_signal_R = 0
                                
        if path_now == 2:
            Velocity_signal_R = 0
            Safe_signal_R = 0
            if right_group["sl"] is not None:
                if center_group["sl"] is not None:
                    X_diffLE_min, Le_V = self.Clac_X_d(center_group, X0, "l", a_ego)
                    X_diffLD_min, Ld_V = self.Clac_X_d(right_group, X0, "l", a_ego)
                    X_diffFE_min, Fe_V = self.Clac_X_d(center_group, X0, "f", a_ego)
                    threshold_F = self.Clac_threshold_F(Fe_V)
                    threshold_L = self.Clac_threshold_L(Le_V)
                    Ld_V = right_group["vl"][0]               
                    if X_diffLE_min >= threshold_L and X_diffLE_min >= th_checking_l and X_diffFE_min >= threshold_F and X_diffFE_min >= th_checking_f:
                        Safe_signal_L = 1
                    else:
                        Safe_signal_L = 0      
                    if round(Ld_V,1) >= round(Le_V,1):
                        Velocity_signal_L = 0
                    else:    
                        Velocity_signal_L = 1    
                    if X_diffLE_min > change_distance:
                        Velocity_signal_L = 1  
                    if X_diffLD_min > change_distance:
                        Velocity_signal_L = 0                      
                if center_group["sl"] is None:
                    Velocity_signal_L = 1    
                    X_diffFE_min, Fe_V = self.Clac_X_d(center_group, X0, "f", a_ego)
                    threshold_F = self.Clac_threshold_F(Fe_V) 
                    if X_diffFE_min >= threshold_F and  X_diffFE_min >= th_checking_f: 
                        Safe_signal_L = 1
                    else:
                        Safe_signal_L = 0                           
            if right_group["sl"] is None:
                Safe_signal_L = 0
                Velocity_signal_L = 0

        if path_now == 1:
            if center_group["sl"] is not None:
                X_diffFU_min, Fu_V = self.Clac_X_d(left_group, X0, "f", a_ego)
                X_diffFD_min, Fd_V = self.Clac_X_d(right_group, X0, "f", a_ego)
                threshold_FU = self.Clac_threshold_F(Fu_V)
                threshold_FD = self.Clac_threshold_F(Fd_V)
                Le_V = ego_group["vl"][0]
                if left_group["sl"] is not None and right_group["sl"] is not None:
                    X_diffLU_min, Lu_V = self.Clac_X_d(left_group, X0, "l", a_ego)
                    X_diffLD_min, Ld_V = self.Clac_X_d(right_group, X0, "l",a_ego)
                    X_diffLE_min, Le_V = self.Clac_X_d(center_group, X0, "l", a_ego)
                    threshold_LU = self.Clac_threshold_L(Lu_V)
                    threshold_LD = self.Clac_threshold_L(Ld_V)
                    if X_diffFU_min >= threshold_FU and X_diffFU_min >= th_checking_f and X_diffLU_min >= threshold_LU and X_diffLU_min >= th_checking_l:
                        Safe_signal_L = 1
                    else:
                        Safe_signal_L = 0
                    if X_diffFD_min >= threshold_FD and X_diffFD_min >= th_checking_f and X_diffLD_min >= threshold_LD and X_diffLD_min >=th_checking_l:
                        Safe_signal_R = 1
                    else:
                        Safe_signal_R = 0
                    if round(Le_V,1) >= round(Ld_V,1):
                        Velocity_signal_R = 0
                    else:
                        Velocity_signal_R = 1
                    if round(Le_V,1) >= round(Lu_V,1):
                        Velocity_signal_L = 0
                    else:
                        Velocity_signal_L = 1
                        
                    if X_diffLU_min > change_distance:
                        Velocity_signal_L = 1
                        
                    if X_diffLD_min > change_distance:
                        Velocity_signal_R = 1 
                            
                    if X_diffLE_min > change_distance:
                        Velocity_signal_L = 0                     
                        Velocity_signal_R = 0    
                                            
                    if Velocity_signal_L and Safe_signal_L and Velocity_signal_R and Safe_signal_R:
                        if round(Lu_V,1) >= round(Ld_V,1):
                            Velocity_signal_R = 0
                        else:
                            Velocity_signal_L = 0
                            
                if left_group["sl"] is None and right_group["sl"] is not None:
                    Velocity_signal_L = 1
                    Velocity_signal_R = 0
                    X_diffLD_min, Ld_V = self.Clac_X_d(right_group, X0, "l",a_ego)
                    threshold_LD = self.Clac_threshold_L(Ld_V)
                    if X_diffFU_min >= threshold_FU:
                        Safe_signal_L = 1
                    else:
                        Safe_signal_L = 0
                    if X_diffFD_min >= threshold_FD and X_diffFD_min >= th_checking_f and  X_diffLD_min >= threshold_LD and X_diffLD_min >=th_checking_l:
                        Safe_signal_R = 1
                    else:
                        Safe_signal_R = 0

                if right_group["sl"] is None and left_group["sl"] is not None:
                    Velocity_signal_R = 1
                    Velocity_signal_L = 0
                    X_diffLU_min, Lu_V = self.Clac_X_d(left_group, X0, "l",a_ego)
                    threshold_LU = self.Clac_threshold_L(Lu_V)
                    if X_diffFD_min >= threshold_FD and X_diffFD_min >= th_checking_f:
                        Safe_signal_R = 1
                    else:
                        Safe_signal_R = 0
                    if X_diffFU_min >= threshold_FU and  X_diffFU_min >= th_checking_f and X_diffLU_min >= threshold_LU and X_diffLU_min >= th_checking_l:
                        Safe_signal_L = 1
                    else:
                        Safe_signal_L = 0
                        
                if left_group["sl"] is None and right_group["sl"] is None:
                    Velocity_signal_L = 1
                    Velocity_signal_R = 1
                    if X_diffFU_min >= threshold_FU and X_diffFU_min >= th_checking_f:
                        Safe_signal_L = 1
                    else:
                        Safe_signal_L = 0
                    
                    if X_diffFD_min >= threshold_FD and X_diffFD_min >= th_checking_f:
                        Safe_signal_R = 1
                    else:
                        Safe_signal_R = 0
                        
                    if Velocity_signal_L and Safe_signal_L and Velocity_signal_R and Safe_signal_R:
                        if threshold_FU <= threshold_FD:
                            Safe_signal_R = 1
                            Safe_signal_L = 0
                        elif threshold_FD <= threshold_FU:
                            Safe_signal_L = 1
                            Safe_signal_R = 0                            
                                        
            if center_group["sl"] is None:
                Safe_signal_L = 0
                Safe_signal_R = 0
                Velocity_signal_L = 0
                Velocity_signal_R = 0
        return Safe_signal_L, Velocity_signal_L, Safe_signal_R, Velocity_signal_R

    #做出并返回direction
    def Decision_Making(self,a_ego,change_distance,group_dict, ego_group,path_now,X0):    
        Safe_signal_L, Velocity_signal_L, Safe_signal_R, Velocity_signal_R= self.Give_signal(a_ego,change_distance,group_dict, ego_group,path_now,X0)   
        print(Safe_signal_L, Velocity_signal_L, Safe_signal_R, Velocity_signal_R)
        if path_now == 0:
            if Safe_signal_R and Velocity_signal_R and X0[0] < self.desired_v:
                Final_signal = -1
                desired_group = group_dict["C1"]
                direction = "Change_down"
            else:
                Final_signal = 0
                desired_group = ego_group
                direction = "Stay"
                
        if path_now == 2:
            if Safe_signal_L and Velocity_signal_L and X0[0] < self.desired_v:
                Final_signal = 1
                desired_group = group_dict["C1"]
                direction = "Change_up"
            else:
                Final_signal = 0
                desired_group = ego_group
                direction = "Stay"
                
        if path_now == 1:
            if Safe_signal_L and Velocity_signal_L and X0[0] < self.desired_v:
                Final_signal = 1
                direction = "Change_up"
                desired_group = group_dict["L1"]
            elif Safe_signal_R and Velocity_signal_R and X0[0] < self.desired_v:
                Final_signal = -1
                direction = "Change_down" 
                desired_group = group_dict["R1"]
            else:
                Final_signal = 0
                direction = "Stay"    
                desired_group = ego_group               

        return desired_group