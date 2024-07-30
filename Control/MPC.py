import sys
sys.path.append(r"C:\Users\sym02\Desktop\Research\Extension\codes\decision_improve") 
import numpy as np
import cvxpy
from casadi import *
from Control.utils import *
from Model.Dynamical_model import *
from Model.params import params
from Control.HOCBF import *

class LMPC:
    def __init__(self,NX,NU,T,TARGET_SPEED,MAX_ACCEL,MIN_ACCEL,
                 MAX_SPEED,MIN_SPEED,MAX_STEER,MIN_STEER,MAX_DACCEL,
                 MIN_DACCEL,MAX_DSTEER,MIN_DSTEER,R,Rd,Q_compare,Q,Qt,P,Pw1,Pw2,Pw3,Pw4,Rs,Rs_f,Rs_tl,Rss,Rssl,Rssr,Rhocbf1,Rhocbf2,
                 slack_a, slack_d,Lane_width,
                 vehicle_width,vehicle_length,MAX_ITER,DU_TH,gamma1,gamma2,gamma_r,
                 a_l,b_l,a_f,b_f,dt,**kwargs):
        
        self.NX = NX
        self.NU = NU
        self.T = T
        self.TARGET_SPEED = TARGET_SPEED
        self.MAX_ACCEL = MAX_ACCEL
        self.MIN_ACCEL = MIN_ACCEL
        self.MAX_SPEED = MAX_SPEED
        self.MIN_SPEED = MIN_SPEED
        self.MAX_STEER = MAX_STEER
        self.MIN_STEER = MIN_STEER
        self.MAX_DACCEL = MAX_DACCEL
        self.MIN_DACCEL = MIN_DACCEL
        self.MAX_DSTEER = MAX_DSTEER
        self.MIN_DSTEER = MIN_DSTEER
        self.R = R                    #对u的权重
        self.Rd = Rd                  #对du的权重
        self.Q = Q                    #对reference track的权重
        self.Q_compare = Q_compare
        self.Qt = Qt                  #对terminal cost的权重
        self.P = P                       #对松弛变量的权重
        self.Pw1 = Pw1
        self.Pw2 = Pw2
        self.Pw3 = Pw3
        self.Pw4 = Pw4 
        self.Rs = Rs                  #对longitudinal constraint的松弛变量
        self.Rs_f = Rs_f
        self.Rs_tl = Rs_tl
        self.Rss = Rss                #对target lane follower constraint的松弛变量
        self.Rssl = Rssl
        self.Rssr = Rssr
        self.Rhocbf1 = Rhocbf1
        self.Rhocbf2 = Rhocbf2
        self.slack_a = slack_a
        self.slack_d = slack_d
        
        self.Lane_width = Lane_width
        self.vehicle_width = vehicle_width
        self.vehicle_length = vehicle_length
        self.add_residual = False
        self.MAX_ITER = MAX_ITER
        self.DU_TH = DU_TH
        
        self.gamma1 = gamma1
        self.gamma2 = gamma2
        self.gamma_r = gamma_r
        self.a_l = a_l
        self.b_l = b_l
        self.a_f = a_f
        self.b_f = b_f
        self.dt = dt
        
############################################ functions for MPC #####################
    def get_refer_path_info(self,path_d,x0,dt):
        s_refer_path = np.zeros(self.T+2)
        x_refer_path = np.zeros(self.T+2)
        psi_refer_path = np.zeros(self.T+2)
        s_refer_path[0] = x0[3]
        x_refer_path[0] = path_d(x0[3])[0]
        psi_refer_path[0] = path_d.get_theta_r(x0[3])
        
        for i in range(self.T+1):
            s_refer_path[i+1] = x0[3] + x0[0]*i*dt
            x_refer_path[i+1] = path_d(s_refer_path[i+1])[0]
            psi_refer_path[i+1] = path_d.get_theta_r(s_refer_path[i+1])
        
        delta_x_list = np.zeros(self.T+1)
        delta_psi_list = np.zeros(self.T+1)
        
        for i in range(self.T+1):
            delta_x_list[i] = x_refer_path[i+1] - x_refer_path[i]
            delta_psi_list[i] = normalize_angle(psi_refer_path[i+1] - psi_refer_path[i])
        return s_refer_path,x_refer_path,psi_refer_path,delta_x_list,delta_psi_list
    
    def get_reference_waypoint(self, x0, path_d, dt):
        s_refer_path,x_refer_path,psi_refer_path,delta_x_list,delta_psi_list = self.get_refer_path_info(path_d,x0,dt)
        reference = np.zeros((6,self.T+1))
        estimation = np.zeros((6,self.T+1))
        reference[:,0] = x0
        estimation[:,0] = x0
        refs = x0[3]
        for i in range(self.T):
            refvx = x0[0]
            refvy = 0.0 
            refw = 0.0 
            refs += x0[0]*dt  
            refey = 0.0 
            refepsi = 0.0 
            reference[0,i+1] = self.TARGET_SPEED
            reference[1,i+1] = refvy
            reference[2,i+1] = refw
            reference[3,i+1] = refs
            reference[4,i+1] = refey
            reference[5,i+1] = refepsi
            
            estimation[0,i+1] = refvx
            if psi_refer_path[i+1] == 0 or psi_refer_path[i+1] == np.pi/2 or psi_refer_path[i+1] == np.pi or psi_refer_path[i+1] == -np.pi/2 or psi_refer_path[i+1] == -np.pi:
                estimation[1,i+1] = 0.0
            else:
                estimation[1,i+1] = refvx/np.tan(psi_refer_path[i+1]) - delta_x_list[i+1]/(dt*np.sin(psi_refer_path[i+1]))
            estimation[2,i+1] = delta_psi_list[i+1]/dt 
            estimation[3,i+1] = refs 
            estimation[4,i+1] = refey 
            estimation[5,i+1] = refepsi 
        return reference, estimation
    
    # Lane-based func
    def get_path_curvature(self, path):
        pathlength = path.get_len()
        s0 = np.arange(0., pathlength, 0.05)
        kapparef = np.zeros_like(s0)
        for i, s in enumerate(s0):
            kapparef[i] = path.get_k(s)
        self.kapparef_s = interpolant("kapparef_s", "bspline", [s0], kapparef)
        return self.kapparef_s
    
    def get_velocity_profile(self, x0, dt):
        speed_profile = np.zeros((self.T,1))
        speed_now = x0[0]
        for i in range(self.T):
            speed_now = speed_now + self.MAX_ACCEL*dt
            speed_profile[i,0] = speed_now
        return speed_profile
    
    
    def get_control_input_profile(self,path_d,x0,dt,ou):
        s_refer_path,x_refer_path,psi_refer_path,delta_x_list,delta_psi_list = self.get_refer_path_info(path_d,x0,dt)
        control_estimation = np.zeros((2,self.T+1))
        control_estimation[0,0] = ou[0]
        control_estimation[1,0] = ou[1]
        for i in range(self.T):
            control_estimation[0,i+1] = 0
            control_estimation[1,i+1] = delta_psi_list[i+1]
        return control_estimation
    

    def get_reference_curv(self, path, x0, ou, dt):
        reference, estimation = self.get_reference_waypoint(x0, path, dt)
        control_estimation =  self.get_control_input_profile(path,x0,dt,ou)
        k_profile  = np.zeros(self.T)
        k_profile[0] = path.get_k(x0[3])
        for i in range(self.T-1):
           k_profile[i+1] = self.kapparef_s(reference[3,i+1])
        return reference, k_profile, control_estimation, estimation
    
    
    def get_GPR_residual(self,estimation,control_estimation,GPR_vy,GPR_w):
        residual_vy = np.zeros((6,self.T))
        residual_w = np.zeros((6,self.T))
        for i in range(self.T):
          y_vy, _ = GPR_vy.predict(estimation[1,i],estimation[2,i])
          y_w, _ = GPR_w.predict(estimation[1,i],estimation[2,i])
          residual_vy[:,i] = y_vy[0]
          residual_w[:,i] = y_w[0]
        return residual_vy, residual_w
    
    def set_util(self,utils):
        self.utils = utils
##########################################################################################
    def MPC_solve(self, path, x0, ou, dt, GPR_vy, GPR_w, label):
        
        reference, k_profile, control_estimation, estimation = self.get_reference_curv(path, x0, ou, dt)

        x = cvxpy.Variable((self.NX, self.T + 1))
        u = cvxpy.Variable((self.NU, self.T))
        d = cvxpy.Variable((2, self.T))

        cost = 0.0
        constraints = []
        
        ugv = Dynamic(**Params)
        
        for t in range(self.T):
            cost += cvxpy.quad_form(u[:, t], self.R)
            cost += cvxpy.quad_form(d[:, t], self.P)
            if t != 0:
                cost += cvxpy.quad_form(reference[:, t] - x[:, t], self.Q)
                
                if label == "K":
                   constraints += [x[4,t] <= 0.8 + d[0,t]]
                   constraints += [x[4,t] + d[1,t] >= -0.8]
                   constraints += [d[:,t]>=0]
                elif label == "L":
                   constraints += [x[4,t] <= 0.8 + d[0,t]]
                   constraints += [x[4,t] + d[1,t] >= -4.3]
                   constraints += [d[:,t]>=0]
                elif label == "R":
                   constraints += [x[4,t] <= 4.3 + d[0,t]]
                   constraints += [x[4,t] + d[1,t] >= -0.8]
                   constraints += [d[:,t]>=0]
            
            A, B, C = ugv.linearized_discretization(estimation[:,t], control_estimation[:,t], k_profile[t], dt)
            
            if self.add_residual:
                residual_vy,residual_w = self.get_GPR_residual(estimation,control_estimation,GPR_vy,GPR_w)
                constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C + residual_vy[:,t] + residual_w[:,t]]
            else:
                constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

            if t < (self.T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], self.Rd)
                constraints += [u[1, t + 1] - u[1, t]<= self.MAX_DSTEER]
                constraints += [u[1, t + 1] - u[1, t]>= -self.MAX_DSTEER]
                constraints += [u[0, t + 1] - u[0, t]<= self.MAX_DACCEL]
        
        cost += cvxpy.quad_form(d[:,self.T-1], self.P)
        cost += cvxpy.quad_form(x[4:, self.T-1], self.Qt)
        cost += cvxpy.quad_form(reference[:, self.T-1] - x[:, self.T-1], self.Q)
        # constraints += [cvxpy.abs(x[4,self.T-1]) <= 0.5*(self.Lane_width - self.vehicle_width)]
        
        
        if label == "K":
            constraints += [x[4,self.T-1] <= 0.8 + d[0,self.T-1]]
            constraints += [x[4,self.T-1] + d[1,self.T-1] >= -0.8]
            constraints += [d[:,self.T-1]>=0]
        elif label == "L":
            constraints += [x[4,self.T-1] <= 0.8 + d[0,self.T-1]]
            constraints += [x[4,self.T-1] + d[1,self.T-1] >= -4.3]
            constraints += [d[:,self.T-1]>=0]
        elif label == "R":
            constraints += [x[4,self.T-1] <= 4.3 + d[0,self.T-1]]
            constraints += [x[4,self.T-1] + d[1,self.T-1] >= -0.8]
            constraints += [d[:,self.T-1]>=0]
            
        constraints += [x[:, 0] == x0[:]]
        constraints += [u[0, :] <= self.MAX_ACCEL]
        constraints += [u[0, :] >= self.MIN_ACCEL]
        constraints += [cvxpy.abs(u[1, :]) <= self.MAX_STEER]


        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.OSQP, verbose=True)
        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            ovx = get_nparray_from_matrix(x.value[0, :])
            ovy = get_nparray_from_matrix(x.value[1, :])
            owz = get_nparray_from_matrix(x.value[2, :])
            os = get_nparray_from_matrix(x.value[3, :])
            oey = get_nparray_from_matrix(x.value[4, :])
            oepsi = get_nparray_from_matrix(x.value[5, :])
            oa = get_nparray_from_matrix(u.value[0, :])
            odelta = get_nparray_from_matrix(u.value[1, :])
        else:
            print("Error: Cannot solve mpc..")
            ovx, ovy, owz, os, oey, oepsi, oa, odelta   = None, None, None, None, None, None, None, None
        return oa, odelta, ovx, ovy, owz, os, oey, oepsi
    
##################################### functions for iMPC-DOHCBF ###########################################
    
    def predict_motion(self, x0, x0_g, oa, od, dt, path_d, last_X):
        #输出的xbar用于estimation
        ovx = last_X[0]
        ovy = last_X[1]
        ow = last_X[2]
        os = last_X[3]
        oey = last_X[4]
        oepsi = last_X[5]
        
        xbar = np.zeros((self.NX, self.T + 1))
        dref = np.zeros((2, self.T + 1))
        k_profile = np.zeros((1, self.T + 1))
        reference = np.zeros((6,self.T+1))
        refs = x0[3]
        for i, _ in enumerate(x0):
            xbar[i, 0] = x0[i]
        
        for i in range(1, self.T + 1):
            xbar[0, i] = ovx[i]
            xbar[1, i] = ovy[i]
            xbar[2, i] = ow[i]
            xbar[3, i] = os[i]
            xbar[4, i] = oey[i]
            xbar[5, i] = oepsi[i]
            dref[1, i] = 0.0
            k_profile[0, i] = path_d.get_k(xbar[3, i])
            
            refvx = 18.0
            refvy = 0.0 
            refw = 0.0 
            refs += x0[0]*dt
            refey = 0.0 
            refepsi = 0.0 
            reference[0,i] = 18.0
            reference[1,i] = refvy
            reference[2,i] = refw
            reference[3,i] = refs
            reference[4,i] = refey
            reference[5,i] = refepsi
            
        return xbar, dref, reference, k_profile
    
    def iterative_linear_mpc_control(self, x0, oa, od, dt, GPR_vy, GPR_w, label, x0_g, path_d,last_X, path_now, ego_group, path_ego, target_group,vehicle_left,vehicle_centre,vehicle_right,path_dindex,C_label_additive,C_label_virtual):
        """
        MPC control with updating operational point iteratively
        """
        # ovx, ovy, ow, os, oey, oepsi, oa, od = None, None, None, None, None, None, None, None
        print("oa",oa is None)
        if oa is None or od is None:
            oa = [0.0] * (self.T)
            od = [0.0] * (self.T)

        for i in range(self.MAX_ITER):   
            oa, od, ovx, ovy, owz, oS, oey, oepsi = self.iMPC_solve_OneStep(path_d, path_dindex,x0, x0_g, oa, od,  GPR_vy, GPR_w, label, C_label_additive,C_label_virtual,last_X, path_now, ego_group, path_ego, target_group,vehicle_left,vehicle_centre,vehicle_right)
            print("oa=",oa)
            print("ob=",od)
            if oa is None:
                print("Solve again!!!!")
                self.Pw1 = np.diag([0.0,0.0])*0.1
                self.Pw2 = np.diag([0.0,0.0])*0.1
                self.b_f = 2.2
                oa = [0.0] * (self.T)
                od = [0.0] * (self.T)               
                # ovx, ovy, owz, oS, oey, oepsi = clac_last_X(oa,od,self.T,path_d,dt,self.NX,x0,x0_g)
                # # last_X = [ovx, ovy, owz, oS, oey, oepsi]
                oa, od, ovx, ovy, owz, oS, oey, oepsi = self.iMPC_solve_OneStep(path_d, path_dindex,x0, x0_g, oa, od, GPR_vy, GPR_w, label,C_label_additive,C_label_virtual,last_X, path_now, ego_group, path_ego, target_group,vehicle_left,vehicle_centre,vehicle_right)
                self.Pw1 = np.diag([5.0,0.0])*0.1
                self.Pw2 = np.diag([5.0,0.0])*0.1
                self.b_f = 2.3
                              
                if oa is None:
                    self.b_f = 2.2
                    self.a_l = 1.3
                    self.Pw1 = np.diag([0.0,0.0])*0.1
                    self.Pw2 = np.diag([0.0,0.0])*0.1
                    oa = [0.0] * (self.T)
                    od = [0.0] * (self.T)               
                    # ovx, ovy, owz, oS, oey, oepsi = clac_last_X(oa,od,self.T,path_d,dt,self.NX,x0,x0_g)
                    # # last_X = [ovx, ovy, owz, oS, oey, oepsi]
                    oa, od, ovx, ovy, owz, oS, oey, oepsi = self.iMPC_solve_OneStep(path_d, path_dindex,x0, x0_g, oa, od, GPR_vy, GPR_w, label,C_label_additive,C_label_virtual,last_X, path_now, ego_group, path_ego, target_group,vehicle_left,vehicle_centre,vehicle_right)
                    self.Pw1 = np.diag([5.0,0.0])*0.1
                    self.Pw2 = np.diag([5.0,0.0])*0.1
                    self.b_f = 2.3
                    self.a_l = 1.5  
        return oa, od, ovx, ovy, owz, oS, oey, oepsi


    def iMPC_solve_OneStep(self, path_d,path_dindex, x0, x0_g, oa, od, GPR_vy, GPR_w, C_label, C_label_additive,C_label_virtual,
                           last_X, path_now, ego_group, path_ego, target_group,vehicle_left,vehicle_centre,vehicle_right):
        
        _, _, _, oS_esti, oey_esti, _ = last_X
        
        x = cvxpy.Variable((self.NX, self.T + 1))
        u = cvxpy.Variable((self.NU, self.T))
        d = cvxpy.Variable((2, self.T))
        w1 = cvxpy.Variable((2, self.T))
        w2 = cvxpy.Variable((2, self.T-1))
        slack_cbf = cvxpy.Variable((2, self.T))
        slack_cbf_f = cvxpy.Variable((2, self.T))
        slack_cbf_tl = cvxpy.Variable((2, self.T))
        slack_surroundl = cvxpy.Variable((2, self.T))
        slack_surroundr = cvxpy.Variable((2, self.T))
        slack_hocbf1 = cvxpy.Variable((2, self.T))
        slack_hocbf2 = cvxpy.Variable((2, self.T-1))
        slackf_hocbf1 = cvxpy.Variable((2, self.T))
        slackf_hocbf2 = cvxpy.Variable((2, self.T-1))
        # slack_acc = cvxpy.Variable((2,self.T))
        # slack_delta = cvxpy.Variable((2,self.T))
        
        if C_label_additive == "constraint":
            prediction_ahead, prediction_rear, prediction_sf_ego, prediction_sl_target, second_paraml, third_paraml, surround_constraints = self.utils.get_all_constraint(C_label,path_now,path_d,ego_group,path_ego,x0,x0_g,target_group,vehicle_left,vehicle_centre,vehicle_right,path_dindex,C_label_virtual)
            # print("prediction_ahead=",prediction_ahead)

            if prediction_ahead is not None:
                a0_l,b0_l,c0_l = tangent_to_ellipse(self.a_l*self.vehicle_length, self.b_l*self.vehicle_width, prediction_ahead[:,0], (x0[3],x0[4]))
                psi_0_0_l = a0_l * x0[3] + b0_l * x0[4] + c0_l # 0's time DHOCBF in this iteration
                
                a1_l,b1_l,c1_l = tangent_to_ellipse(self.a_l*self.vehicle_length, self.b_l*self.vehicle_width, prediction_ahead[:,1], [oS_esti[2],oey_esti[2]])
                psi_0_1_l = a1_l * x[3,1] + b1_l * x[4,1] + c1_l        
            
            if prediction_rear is not None:
                print("prediction_rear[:,0]=",prediction_rear[:,0])
                print("(x0[3],x0[4])=",(x0[3],x0[4]))
                a0_f,b0_f,c0_f = tangent_to_ellipse(self.a_f*self.vehicle_length, self.b_f*self.vehicle_width, prediction_rear[:,0], (x0[3],x0[4]))
                psi_0_0_f = a0_f * x0[3] + b0_f * x0[4] + c0_f # 0's time DHOCBF in this iteration
                
                a1_f,b1_f,c1_f = tangent_to_ellipse(self.a_f*self.vehicle_length, self.b_f*self.vehicle_width, prediction_rear[:,1], [oS_esti[2],oey_esti[2]])
                psi_0_1_f = a1_f * x[3,1] + b1_f * x[4,1] + c1_f              
        
        elif C_label_additive == "Probe":
            prediction_ahead, prediction_sf_ego, second_paraml, third_paraml, surround_constraints = self.utils.get_all_constraint(C_label,path_now,path_d,ego_group,path_ego,x0,x0_g,target_group,vehicle_left,vehicle_centre,vehicle_right,path_dindex,C_label_virtual)
            print("surround_constraints=",surround_constraints)
            # print("prediction_ahead=",prediction_ahead)
            if prediction_ahead is not None:
                a0_l,b0_l,c0_l = tangent_to_ellipse(self.a_l*self.vehicle_length, self.b_l*self.vehicle_width, prediction_ahead[:,0], (x0[3],x0[4]))
                psi_0_0_l = a0_l * x0[3] + b0_l * x0[4] + c0_l # 0's time DHOCBF in this iteration
                
                a1_l,b1_l,c1_l = tangent_to_ellipse(self.a_l*self.vehicle_length, self.b_l*self.vehicle_width, prediction_ahead[:,1], [oS_esti[2],oey_esti[2]])
                psi_0_1_l = a1_l * x[3,1] + b1_l * x[4,1] + c1_l   
                
        elif C_label_additive == "No Probe":
            prediction_sl_ego, prediction_sf_ego, second_paraml, third_paraml, surround_constraints = self.utils.get_all_constraint(C_label,path_now,path_d,ego_group,path_ego,x0,x0_g,target_group,vehicle_left,vehicle_centre,vehicle_right,path_dindex,C_label_virtual)
            print("surround_constraints=",surround_constraints)
        xbar, dref, reference, k_profile = self.predict_motion(x0, x0_g, oa, od, self.dt, path_d, last_X)
        
        cost = 0.0
        constraints = []
        
        ugv = Dynamic(**Params)
        
        for t in range(self.T):

            cost += cvxpy.quad_form(u[:, t], self.R)
        
            
            #对于 follower 的约束
            if prediction_sf_ego is not None:
                cost += cvxpy.quad_form(slack_cbf_f[:, t], self.Rs)
                ht = x[3,t] - prediction_sf_ego[t]-third_paraml[t]-self.vehicle_length
                htplus1 = x[3,t+1] - prediction_sf_ego[t+1]-third_paraml[t+1]-self.vehicle_length
                Delta_h = htplus1 - ht
                constraints += [Delta_h + self.gamma_r*ht + slack_cbf_f[0,t]>= 0]

            if t != 0:
                cost += cvxpy.quad_form(reference[:, t] - x[:, t], self.Q[t])
                cost += cvxpy.quad_form(u[:, t] - u[:, t-1], self.Rd)
                # constraints for lane restriction
                if C_label_virtual == "K":
                    if path_now == 0:
                        constraints += [x[4,t] <= 4.5]
                    elif path_now == 2:
                        constraints += [x[4,t] >= -4.5]
                    elif path_now == 1:
                        cost += cvxpy.quad_form(d[:, t], self.P)
                        constraints += [x[4,t] <= 0.01 + d[0,t]]
                        constraints += [x[4,t] + d[1,t] >= -0.01]

                    
                elif C_label_virtual == "L":
                    if path_now == 2:
                        constraints += [x[4,t] >= -4.5]
                    
                elif  C_label_virtual == "R":
                    if path_now == 0:
                        constraints += [x[4,t] <= 4.5]
                
                if C_label_additive == "constraint":
                    if prediction_ahead is not None and t <=20:
                        cost += cvxpy.quad_form(([1,0]-w1[:,t]), self.Pw1)
                        cost += cvxpy.quad_form(slack_hocbf1[:,t], self.Rhocbf1)
                        at_l,bt_l,ct_l = tangent_to_ellipse(self.a_l*self.vehicle_length, self.b_l*self.vehicle_width, prediction_ahead[0:2,t], [oS_esti[t+1],oey_esti[t+1]])
                        psi_t_l = at_l * x[3,t] + bt_l * x[4,t] + ct_l
                        constraints += [psi_t_l + slack_hocbf1[0,t]>= w1[0,t]*(1-self.gamma1)**t * psi_0_0_l]
                        # constraints += [psi_t_l >= w1[0,t]*(1-self.gamma1)**t * psi_0_0_l]
                        
                        if  t <20 and t > 0:
                            cost += cvxpy.quad_form(([1,0]-w2[:,t]), self.Pw2)
                            cost += cvxpy.quad_form(slack_hocbf2[:,t], self.Rhocbf2)
                            at_l_1,bt_l_1,ct_l_1 = tangent_to_ellipse(self.a_l*self.vehicle_length, self.b_l*self.vehicle_width, prediction_ahead[0:2,t], [oS_esti[t+1],oey_esti[t+1]])
                            psi_0_tplus1_l = at_l_1 * x[3,t+1] + bt_l_1 * x[4,t+1] + ct_l_1
                            constraints += [psi_0_tplus1_l + (self.gamma1-1)*psi_t_l - (1-self.gamma2)**t * psi_0_1_l + slack_hocbf2[0,t]>= w2[0,t]*(self.gamma1-1)*(1-self.gamma2)**t*psi_0_0_l]
                            constraints += [psi_0_tplus1_l + (self.gamma1-1)*psi_t_l - (1-self.gamma2)**t * psi_0_1_l >= w2[0,t]*(self.gamma1-1)*(1-self.gamma2)**t*psi_0_0_l]

                    if prediction_rear is not None and t <= 20:
                        cost += cvxpy.quad_form(([1,0]-w1[:,t]), self.Pw1)
                        cost += cvxpy.quad_form(slackf_hocbf1[:,t], self.Rhocbf1)
                        at_f,bt_f,ct_f = tangent_to_ellipse(self.a_f*self.vehicle_length, self.b_f*self.vehicle_width, prediction_rear[0:2,t], [oS_esti[t+1],oey_esti[t+1]])
                        psi_t_f = at_f * x[3,t] + bt_f * x[4,t] + ct_f
                        constraints += [psi_t_f + slackf_hocbf1[0,t]>= w1[0,t]*(1-self.gamma1)**t * psi_0_0_f]
                        # constraints += [psi_t_f >= w1[0,t]*(1-self.gamma1)**t * psi_0_0_f]
                        
                        if t < 20 and t > 0:
                            cost += cvxpy.quad_form(([1,0]-w2[:,t]), self.Pw2)
                            cost += cvxpy.quad_form(slackf_hocbf2[:,t], self.Rhocbf2)
                            at_f_1,bt_f_1,ct_f_1 = tangent_to_ellipse(self.a_f*self.vehicle_length, self.b_f*self.vehicle_width, prediction_rear[0:2,t], [oS_esti[t+1],oey_esti[t+1]])
                            psi_0_tplus1_f = at_f_1 * x[3,t+1] + bt_f_1 * x[4,t+1] + ct_f_1
                            constraints += [psi_0_tplus1_f + (self.gamma1-1)*psi_t_f - (1-self.gamma2)**t * psi_0_1_f + slackf_hocbf2[0,t]>= w2[0,t]*(self.gamma1-1)*(1-self.gamma2)**t*psi_0_0_f]
                            constraints += [psi_0_tplus1_f + (self.gamma1-1)*psi_t_f - (1-self.gamma2)**t * psi_0_1_f >= w2[0,t]*(self.gamma1-1)*(1-self.gamma2)**t*psi_0_0_f]
                    
                    if prediction_sl_target is not None:
                        cost += cvxpy.quad_form(slack_cbf_tl[:, t], self.Rs)
                        ht_t = prediction_sl_target[t]-x[3,t]-0.5*x[0,t]-5.0-self.vehicle_length
                        htplus1_t = prediction_sl_target[t+1]-x[3,t+1]-0.5*x[0,t+1]-5.0-self.vehicle_length
                        Delta_h_t = htplus1_t - ht_t
                        constraints += [Delta_h_t + self.gamma_r*ht_t + slack_cbf_tl[0,t]>= 0]

                if C_label_additive == "Probe":
                    if prediction_ahead is not None and t <=20:
                        cost += cvxpy.quad_form(([1,0]-w1[:,t]), self.Pw1)
                        cost += cvxpy.quad_form(slack_hocbf1[:,t], self.Rhocbf1)
                        at_l,bt_l,ct_l = tangent_to_ellipse(self.a_l*self.vehicle_length, self.b_l*self.vehicle_width, prediction_ahead[0:2,t], [oS_esti[t+1],oey_esti[t+1]])
                        psi_t_l = at_l * x[3,t] + bt_l * x[4,t] + ct_l
                        constraints += [psi_t_l + slack_hocbf1[0,t]>= w1[0,t]*(1-self.gamma1)**t * psi_0_0_l]
                        # constraints += [psi_t_l >= w1[0,t]*(1-self.gamma1)**t * psi_0_0_l]
                        
                        if t < 20 and t > 0:
                            cost += cvxpy.quad_form(([1,0]-w2[:,t]), self.Pw2)
                            cost += cvxpy.quad_form(slack_hocbf2[:,t], self.Rhocbf2)
                            at_l_1,bt_l_1,ct_l_1 = tangent_to_ellipse(self.a_l*self.vehicle_length, self.b_l*self.vehicle_width, prediction_ahead[0:2,t], [oS_esti[t+1],oey_esti[t+1]])
                            psi_0_tplus1_l = at_l_1 * x[3,t+1] + bt_l_1 * x[4,t+1] + ct_l_1
                            constraints += [psi_0_tplus1_l + (self.gamma1-1)*psi_t_l - (1-self.gamma2)**t * psi_0_1_l + slack_hocbf2[0,t]>= w2[0,t]*(self.gamma1-1)*(1-self.gamma2)**t*psi_0_0_l]
                            # constraints += [psi_0_tplus1_l + (self.gamma1-1)*psi_t_l - (1-self.gamma2)**t * psi_0_1_l >= w2[0,t]*(self.gamma1-1)*(1-self.gamma2)**t*psi_0_0_l]
                                        
            # longitudinal constraints in ego lane
            if C_label == "K":
                if prediction_sl_ego is not None:
                    cost += cvxpy.quad_form(slack_cbf[:, t], self.Rs)
                    ht = prediction_sl_ego[t]-x[3,t]-second_paraml[t]*x[0,t]-third_paraml[t]-self.vehicle_length
                    htplus1 = prediction_sl_ego[t+1]-x[3,t+1]-second_paraml[t+1]*x[0,t+1]-third_paraml[t+1]-self.vehicle_length
                    Delta_h = htplus1 - ht
                    constraints += [Delta_h + self.gamma_r*ht + slack_cbf[0,t]>= 0]

            # surround constraints in other lanes
            if path_now == 0:
                dR_min_surround = surround_constraints
                if dR_min_surround is not None:
                    cost += cvxpy.quad_form(slack_surroundr[:,t], self.Rssr)
                    constraints += [x[4,t] + slack_surroundr[0,t] >= dR_min_surround[t]]

            elif path_now == 1:
                dL_min_surround,dR_min_surround = surround_constraints
                if dL_min_surround is not None:
                    cost += cvxpy.quad_form(slack_surroundl[:,t], self.Rssl)
                    constraints += [x[4,t] + slack_surroundl[0,t]<= dL_min_surround[t]]
     
                if dR_min_surround is not None:
                    cost += cvxpy.quad_form(slack_surroundr[:,t], self.Rssr)
                    constraints += [x[4,t] + slack_surroundr[0,t]>= dR_min_surround[t]]
  
            elif path_now == 2:
                dL_min_surround = surround_constraints
                if dL_min_surround is not None:
                    cost += cvxpy.quad_form(slack_surroundl[:,t], self.Rssl)
                    constraints += [x[4,t] + slack_surroundl[0,t]<= dL_min_surround[t]]
                    
 
            A, B, C = ugv.linearized_discretization(xbar[:,t], dref[:,t], k_profile[0,t], self.dt)
            
            if self.add_residual:
                residual_vy,residual_w = self.get_GPR_residual(xbar,dref,GPR_vy,GPR_w)
                constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C + residual_vy[:,t] + residual_w[:,t]]
            else:
                constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

            if t < (self.T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], self.Rd)
                constraints += [u[1, t + 1] - u[1, t]>= self.MIN_DSTEER * self.DU_TH]
                constraints += [u[0, t + 1] - u[0, t] <= self.MAX_DACCEL * self.DU_TH]
                constraints += [u[0, t + 1] - u[0, t] >= self.MIN_DACCEL * self.DU_TH]
        
        cost += cvxpy.quad_form(d[:,self.T-1], self.P)
        cost += cvxpy.quad_form(reference[:, self.T-1] - x[:, self.T], self.Q[self.T-1])
        cost += cvxpy.quad_form(x[4:, self.T], self.Qt)
        
        # surround constraints in other lanes
        if path_now == 0:
            dR_min_surround = surround_constraints
            if dR_min_surround is not None:
                cost += cvxpy.quad_form(slack_surroundr[:,self.T-1], self.Rssr)
                constraints += [x[4,self.T] + slack_surroundr[0,self.T-1] >= dR_min_surround[self.T]]

        elif path_now == 1:
            dL_min_surround,dR_min_surround = surround_constraints
            if dL_min_surround is not None:
                cost += cvxpy.quad_form(slack_surroundl[:,self.T-1], self.Rssl)
                constraints += [x[4,self.T] + slack_surroundl[0,self.T-1]<= dL_min_surround[self.T]]
                
            if dR_min_surround is not None:
                cost += cvxpy.quad_form(slack_surroundr[:,self.T-1], self.Rssr)
                constraints += [x[4,self.T] + slack_surroundr[0,self.T-1]>= dR_min_surround[self.T]]
                
        elif path_now == 2:
            dL_min_surround = surround_constraints
            print("dL_min_surround=",dL_min_surround)
            if dL_min_surround is not None:
                cost += cvxpy.quad_form(slack_surroundl[:,self.T-1], self.Rssl)
                constraints += [x[4,self.T] + slack_surroundl[0,self.T-1]<= dL_min_surround[self.T]]
                constraints += [slack_surroundl[0,self.T-1] >=0]
        
        # constraints for lane restriction
        if  C_label_virtual == "K":
            if path_now == 0:
                constraints += [x[4,self.T] <= 4.5]
            elif path_now == 2:
                constraints += [x[4,self.T] >= -4.5]
            elif path_now == 1:
                cost += cvxpy.quad_form(d[:, self.T-1], self.P)
                constraints += [x[4,self.T] <= 0.01 + d[0,self.T-1]]
                constraints += [x[4,self.T] + d[1,self.T-1] >= -0.01]

            
        elif C_label_virtual == "L":
                if path_now == 2:
                    constraints += [x[4,self.T] >= -4.5]

            
        elif C_label_virtual == "R":
                if path_now == 0:
                    constraints += [x[4,self.T] <= 4.5]

        
        constraints += [x[:, 0] == x0[:]]
        constraints += [u[0, :] <= self.MAX_ACCEL ]
        constraints += [u[0, :] >= self.MIN_ACCEL]
        # constraints += [slack_acc[:,:] >=0]
        # constraints += [slack_delta[:,:] >=0]
        constraints += [cvxpy.abs(u[1, :]) <= self.MAX_STEER]
        # constraints += [x[5,:] >= -np.pi]
        # constraints += [x[5,:] <= np.pi]
        
        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        try:            
            prob.solve(solver=cvxpy.ECOS, verbose=False)
        except:
            print("Error: Cannot solve mpc..")
            ovx, ovy, owz, os, oey, oepsi, oa, odelta   = None, None, None, None, None, None, None, None            
        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            ovx = get_nparray_from_matrix(x.value[0, :])
            ovy = get_nparray_from_matrix(x.value[1, :])
            owz = get_nparray_from_matrix(x.value[2, :])
            os = get_nparray_from_matrix(x.value[3, :])
            oey = get_nparray_from_matrix(x.value[4, :])
            oepsi = get_nparray_from_matrix(x.value[5, :])
            oa = get_nparray_from_matrix(u.value[0, :])
            odelta = get_nparray_from_matrix(u.value[1, :])
        else:
            print("Error: Cannot solve mpc..")
            ovx, ovy, owz, os, oey, oepsi, oa, odelta   = None, None, None, None, None, None, None, None
        return oa, odelta, ovx, ovy, owz, os, oey, oepsi
                      

    def iterative_linear_mpc_control_for_comparison(self, x0, oa, od, dt, GPR_vy, GPR_w, label, x0_g, path_d,last_X, path_now, ego_group, path_ego, target_group,vehicle_left,vehicle_centre,vehicle_right,path_dindex):
        """
        MPC control with updating operational point iteratively
        """
        ovx, ovy, ow, os, oey, oepsi, oa, od = None, None, None, None, None, None, None, None

        if oa is None or od is None:
            oa = [0.0] * (self.T)
            od = [0.0] * (self.T)

        for i in range(self.MAX_ITER):   
            oa, od, ovx, ovy, owz, oS, oey, oepsi = self.iMPC_solve_for_comparison(path_d, path_dindex,x0, x0_g, oa, od,  GPR_vy, GPR_w, label, last_X, path_now, ego_group, path_ego, target_group,vehicle_left,vehicle_centre,vehicle_right)
            if oa is None:
                print("Solve again!!!!")
                oa = [0.0] * (self.T)
                od = [0.0] * (self.T)               
                ovx, ovy, owz, oS, oey, oepsi = clac_last_X(oa,od,self.T,path_d,dt,self.NX,x0,x0_g)
                last_X = [ovx, ovy, owz, oS, oey, oepsi]
                oa, od, ovx, ovy, owz, oS, oey, oepsi = self.iMPC_solve_for_comparison(path_d, path_dindex,x0, x0_g, oa, od,  GPR_vy, GPR_w, label,last_X, path_now, ego_group, path_ego, target_group,vehicle_left,vehicle_centre,vehicle_right)
            # du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
            # if du <= self.DU_TH:
            #     break
        else:
            print("Iterative is max iter")
        return oa, od, ovx, ovy, owz, oS, oey, oepsi

    def iMPC_solve_for_comparison(self, path_d,path_dindex, x0, x0_g, oa, od, GPR_vy, GPR_w, C_label,
                           last_X, path_now, ego_group, path_ego, target_group,vehicle_left,vehicle_centre,vehicle_right):
        
        print("starts to solve here")
        x = cvxpy.Variable((self.NX, self.T + 1))
        u = cvxpy.Variable((self.NU, self.T))
        d = cvxpy.Variable((2, self.T))
        slack_cbf = cvxpy.Variable((2, self.T))
        slack_cbf_f = cvxpy.Variable((2, self.T))
        slack_surroundl = cvxpy.Variable((2, self.T))
        slack_surroundr = cvxpy.Variable((2, self.T))
                                                                                                                                        
        prediction_sl_ego, prediction_sf_ego, second_paraml, third_paraml, surround_constraints = self.utils.get_all_constraint_for_comparison(path_now,path_d,ego_group,path_ego,x0_g,target_group,vehicle_left,vehicle_centre,vehicle_right,path_dindex,C_label)

        xbar, dref, reference, k_profile = self.predict_motion(x0, x0_g, oa, od, self.dt, path_d, last_X)
        
        cost = 0.0
        constraints = []
        
        ugv = Dynamic(**Params)
        
        print("enter for")
        for t in range(self.T):
            cost += cvxpy.quad_form(u[:, t], self.R)
            cost += cvxpy.quad_form(d[:, t], self.P)
            cost += cvxpy.quad_form(slack_cbf_f[:, t], self.Rs)
            
            #对于 follower 的约束
            if prediction_sf_ego is not None:
                ht = x[3,t] - prediction_sf_ego[t]-third_paraml[t]-self.vehicle_length
                htplus1 = x[3,t+1] - prediction_sf_ego[t+1]-third_paraml[t+1]-self.vehicle_length
                Delta_h = htplus1 - ht
                constraints += [Delta_h + self.gamma_r*ht + slack_cbf_f[0,t]>= 0]

            if t != 0:
                print("")
                cost += cvxpy.quad_form(reference[:, t] - x[:, t], self.Q_compare)
                cost += cvxpy.quad_form(u[:, t] - u[:, t-1], self.Rd)

            # longitudinal constraints in ego lane

            if prediction_sl_ego is not None:
                cost += cvxpy.quad_form(slack_cbf[:, t], self.Rs)
                ht = prediction_sl_ego[t]-x[3,t]-second_paraml[t]*x[0,t]-third_paraml[t]-self.vehicle_length
                htplus1 = prediction_sl_ego[t+1]-x[3,t+1]-second_paraml[t+1]*x[0,t+1]-third_paraml[t+1]-self.vehicle_length
                Delta_h = htplus1 - ht
                constraints += [Delta_h + self.gamma_r*ht + slack_cbf[0,t]>= 0]

            # surround constraints in other lanes
            if path_now == 0:
                dR_min_surround = surround_constraints
                if dR_min_surround is not None:
                    cost += cvxpy.quad_form(slack_surroundr[:,t], self.Rssr)
                    constraints += [x[4,t] + slack_surroundr[0,t] >= dR_min_surround[t]]

            elif path_now == 1:
                dL_min_surround,dR_min_surround = surround_constraints
                if dL_min_surround is not None:
                    cost += cvxpy.quad_form(slack_surroundl[:,t], self.Rssl)
                    constraints += [x[4,t] + slack_surroundl[0,t]<= dL_min_surround[t]]
     
                if dR_min_surround is not None:
                    cost += cvxpy.quad_form(slack_surroundr[:,t], self.Rssr)
                    constraints += [x[4,t] + slack_surroundr[0,t]>= dR_min_surround[t]]
  
            elif path_now == 2:
                dL_min_surround = surround_constraints
                if dL_min_surround is not None:
                    cost += cvxpy.quad_form(slack_surroundl[:,t], self.Rssl)
                    constraints += [x[4,t] + slack_surroundl[0,t]<= dL_min_surround[t]]

                # constraints for lane restriction
                if C_label == "K":
                   constraints += [x[4,t] <= 0.01 + d[0,t]]
                   constraints += [x[4,t] + d[1,t] >= -0.01]
                   
                elif C_label == "L":
                   constraints += [x[4,t] <= -3.4 + d[0,t]]
                   
                elif  C_label == "R":
                   constraints += [x[4,t] + d[0,t] >= 3.4]

 
            A, B, C = ugv.linearized_discretization(xbar[:,t], dref[:,t], k_profile[0,t], self.dt)
            
            if self.add_residual:
                residual_vy,residual_w = self.get_GPR_residual(xbar,dref,GPR_vy,GPR_w)
                constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C + residual_vy[:,t] + residual_w[:,t]]
            else:
                constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

            if t < (self.T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], self.Rd)
                constraints += [u[1, t + 1] - u[1, t]>= self.MIN_DSTEER * self.DU_TH]
                constraints += [u[0, t + 1] - u[0, t] <= self.MAX_DACCEL * self.DU_TH]
                constraints += [u[0, t + 1] - u[0, t] >= self.MIN_DACCEL * self.DU_TH]
        
        cost += cvxpy.quad_form(d[:,self.T-1], self.P)
        cost += cvxpy.quad_form(reference[:, self.T-1] - x[:, self.T], self.Q_compare)
        cost += cvxpy.quad_form(x[4:, self.T], self.Qt)
        
        # surround constraints in other lanes
        if path_now == 0:
            dR_min_surround = surround_constraints
            if dR_min_surround is not None:
                cost += cvxpy.quad_form(slack_surroundr[:,self.T-1], self.Rssr)
                constraints += [x[4,self.T] + slack_surroundr[0,self.T-1] >= dR_min_surround[self.T]]

        elif path_now == 1:
            dL_min_surround,dR_min_surround = surround_constraints
            if dL_min_surround is not None:
                cost += cvxpy.quad_form(slack_surroundl[:,self.T-1], self.Rssl)
                constraints += [x[4,self.T] + slack_surroundl[1,self.T-1]<= dL_min_surround[self.T]]
                
            if dR_min_surround is not None:
                cost += cvxpy.quad_form(slack_surroundr[:,self.T-1], self.Rssr)
                constraints += [x[4,self.T] + slack_surroundr[0,self.T-1]>= dR_min_surround[self.T]]
                
        elif path_now == 2:
            dL_min_surround = surround_constraints
            if dL_min_surround is not None:
                cost += cvxpy.quad_form(slack_surroundl[:,self.T-1], self.Rssl)
                constraints += [x[4,self.T] + slack_surroundl[1,self.T-1]<= dL_min_surround[self.T]]

        # constraints for lane restriction
        if  C_label == "K":
            constraints += [x[4,self.T] <= 0.01 + d[0,self.T-1]]
            constraints += [x[4,self.T] + d[1,self.T-1] >= -0.01]
            
        elif C_label == "L":
            constraints += [x[4,self.T]  <= -3.4 + d[0,self.T-1]]

        elif C_label == "R":
            constraints += [x[4,self.T] + d[0,self.T-1] >= 3.4]

        constraints += [x[:, 0] == x0[:]]
        constraints += [u[0, :] <= self.MAX_ACCEL ]
        constraints += [u[0, :] >= self.MIN_ACCEL]
        constraints += [cvxpy.abs(u[1, :]) <= self.MAX_STEER]

        
        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.ECOS, verbose=False)
        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            ovx = get_nparray_from_matrix(x.value[0, :])
            ovy = get_nparray_from_matrix(x.value[1, :])
            owz = get_nparray_from_matrix(x.value[2, :])
            os = get_nparray_from_matrix(x.value[3, :])
            oey = get_nparray_from_matrix(x.value[4, :])
            oepsi = get_nparray_from_matrix(x.value[5, :])
            oa = get_nparray_from_matrix(u.value[0, :])
            odelta = get_nparray_from_matrix(u.value[1, :])
        else:
            print("Error: Cannot solve mpc..")
            ovx, ovy, owz, os, oey, oepsi, oa, odelta   = None, None, None, None, None, None, None, None
        return oa, odelta, ovx, ovy, owz, os, oey, oepsi
    
    
    
    def iterative_linear_mpc_control_for_noadapt(self, x0, oa, od, dt, GPR_vy, GPR_w, label, x0_g, path_d,last_X, path_now, ego_group, path_ego, target_group,vehicle_left,vehicle_centre,vehicle_right,path_dindex,C_label_additive,C_label_virtual):
        """
        MPC control with updating operational point iteratively
        """
        ovx, ovy, ow, os, oey, oepsi, oa, od = None, None, None, None, None, None, None, None

        if oa is None or od is None:
            oa = [0.0] * (self.T)
            od = [0.0] * (self.T)

        for i in range(self.MAX_ITER):   
            oa, od, ovx, ovy, owz, oS, oey, oepsi = self.iMPC_solve_OneStep_for_noadapt(path_d, path_dindex,x0, x0_g, oa, od,  GPR_vy, GPR_w, label, C_label_additive,C_label_virtual,last_X, path_now, ego_group, path_ego, target_group,vehicle_left,vehicle_centre,vehicle_right)
            if oa is None:
                print("Solve again!!!!")
                oa = [0.0] * (self.T)
                od = [0.0] * (self.T)               
                ovx, ovy, owz, oS, oey, oepsi = clac_last_X(oa,od,self.T,path_d,dt,self.NX,x0,x0_g)
                last_X = [ovx, ovy, owz, oS, oey, oepsi]
                oa, od, ovx, ovy, owz, oS, oey, oepsi = self.iMPC_solve_OneStep_for_noadapt(path_d, path_dindex,x0, x0_g, oa, od,  GPR_vy, GPR_w, label,C_label_additive,C_label_virtual,last_X, path_now, ego_group, path_ego, target_group,vehicle_left,vehicle_centre,vehicle_right)
            # du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
            # if du <= self.DU_TH:
            #     break
        else:
            print("Iterative is max iter")

        return oa, od, ovx, ovy, owz, oS, oey, oepsi


    def iMPC_solve_OneStep_for_noadapt(self, path_d,path_dindex, x0, x0_g, oa, od, GPR_vy, GPR_w, C_label, C_label_additive,C_label_virtual,
                           last_X, path_now, ego_group, path_ego, target_group,vehicle_left,vehicle_centre,vehicle_right):
        
        _, _, _, oS_esti, oey_esti, _ = last_X
        
        x = cvxpy.Variable((self.NX, self.T + 1))
        u = cvxpy.Variable((self.NU, self.T))
        d = cvxpy.Variable((2, self.T))
        w1 = cvxpy.Variable((2, self.T))
        w2 = cvxpy.Variable((2, self.T-1))
        slack_cbf = cvxpy.Variable((2, self.T))
        slack_cbf_f = cvxpy.Variable((2, self.T))
        slack_cbf_tl = cvxpy.Variable((2, self.T))
        slack_surroundl = cvxpy.Variable((2, self.T))
        slack_surroundr = cvxpy.Variable((2, self.T))
        slack_hocbf1 = cvxpy.Variable((2, self.T))
        slack_hocbf2 = cvxpy.Variable((2, self.T-1))
        slackf_hocbf1 = cvxpy.Variable((2, self.T))
        slackf_hocbf2 = cvxpy.Variable((2, self.T-1))
        # slack_acc = cvxpy.Variable((2,self.T))
        # slack_delta = cvxpy.Variable((2,self.T))
        
        if C_label_additive == "constraint":
            prediction_ahead, prediction_rear, prediction_sf_ego, prediction_sl_target, second_paraml, third_paraml, surround_constraints = self.utils.get_all_constraint_for_noadapt(C_label,path_now,path_d,ego_group,path_ego,x0,x0_g,target_group,vehicle_left,vehicle_centre,vehicle_right,path_dindex,C_label_virtual)
            print("surround_constraints=",surround_constraints)

            if prediction_ahead is not None:
                a0_l,b0_l,c0_l = tangent_to_ellipse(self.a_l*self.vehicle_length, self.b_l*self.vehicle_width, prediction_ahead[:,0], (x0[3],x0[4]))
                psi_0_0_l = a0_l * x0[3] + b0_l * x0[4] + c0_l # 0's time DHOCBF in this iteration
                
                a1_l,b1_l,c1_l = tangent_to_ellipse(self.a_l*self.vehicle_length, self.b_l*self.vehicle_width, prediction_ahead[:,1], [oS_esti[1],oey_esti[1]])
                psi_0_1_l = a1_l * x[3,1] + b1_l * x[4,1] + c1_l        
            
            if prediction_rear is not None:
                a0_f,b0_f,c0_f = tangent_to_ellipse(self.a_f*self.vehicle_length, self.b_f*self.vehicle_width, prediction_rear[:,0], (x0[3],x0[4]))
                psi_0_0_f = a0_f * x0[3] + b0_f * x0[4] + c0_f # 0's time DHOCBF in this iteration
                
                a1_f,b1_f,c1_f = tangent_to_ellipse(self.a_f*self.vehicle_length, self.b_f*self.vehicle_width, prediction_rear[:,1], [oS_esti[1],oey_esti[1]])
                psi_0_1_f = a1_f * x[3,1] + b1_f * x[4,1] + c1_f              
        
        elif C_label_additive == "Probe":
            prediction_ahead, prediction_sf_ego, second_paraml, third_paraml, surround_constraints = self.utils.get_all_constraint_for_noadapt(C_label,path_now,path_d,ego_group,path_ego,x0,x0_g,target_group,vehicle_left,vehicle_centre,vehicle_right,path_dindex,C_label_virtual)
            print("surround_constraints=",surround_constraints)
            # print("prediction_ahead=",prediction_ahead)
            if prediction_ahead is not None:
                a0_l,b0_l,c0_l = tangent_to_ellipse(self.a_l*self.vehicle_length, self.b_l*self.vehicle_width, prediction_ahead[:,0], (x0[3],x0[4]))
                psi_0_0_l = a0_l * x0[3] + b0_l * x0[4] + c0_l # 0's time DHOCBF in this iteration
                
                a1_l,b1_l,c1_l = tangent_to_ellipse(self.a_l*self.vehicle_length, self.b_l*self.vehicle_width, prediction_ahead[:,1], [oS_esti[1],oey_esti[1]])
                psi_0_1_l = a1_l * x[3,1] + b1_l * x[4,1] + c1_l   
                
        elif C_label_additive == "No Probe":
            prediction_sl_ego, prediction_sf_ego, second_paraml, third_paraml, surround_constraints = self.utils.get_all_constraint_for_noadapt(C_label,path_now,path_d,ego_group,path_ego,x0,x0_g,target_group,vehicle_left,vehicle_centre,vehicle_right,path_dindex,C_label_virtual)
            print("surround_constraints=",surround_constraints)
        xbar, dref, reference, k_profile = self.predict_motion(x0, x0_g, oa, od, self.dt, path_d, last_X)
        
        cost = 0.0
        constraints = []
        
        ugv = Dynamic(**Params)
        
        for t in range(self.T):

            cost += cvxpy.quad_form(u[:, t], self.R)
            cost += cvxpy.quad_form(d[:, t], self.P)
        
            
            #对于 follower 的约束
            if prediction_sf_ego is not None:
                cost += cvxpy.quad_form(slack_cbf_f[:, t], self.Rs)
                ht = x[3,t] - prediction_sf_ego[t]-third_paraml[t]-self.vehicle_length
                htplus1 = x[3,t+1] - prediction_sf_ego[t+1]-third_paraml[t+1]-self.vehicle_length
                Delta_h = htplus1 - ht
                constraints += [Delta_h + self.gamma_r*ht + slack_cbf_f[0,t]>= 0]

            if t != 0:
                cost += cvxpy.quad_form(reference[:, t] - x[:, t], self.Q[t])
                cost += cvxpy.quad_form(u[:, t] - u[:, t-1], self.Rd)

                if C_label_additive == "constraint":
                    if prediction_ahead is not None and t <=15:
                        cost += cvxpy.quad_form(([1,0]-w1[:,t]), self.Pw1)
                        cost += cvxpy.quad_form(slack_hocbf1[:,t], self.Rhocbf1)
                        at_l,bt_l,ct_l = tangent_to_ellipse(self.a_l*self.vehicle_length, self.b_l*self.vehicle_width, prediction_ahead[0:2,t], [oS_esti[t],oey_esti[t]])
                        psi_t_l = at_l * x[3,t] + bt_l * x[4,t] + ct_l
                        constraints += [psi_t_l + slack_hocbf1[0,t]>= w1[0,t]*(1-self.gamma1)**t * psi_0_0_l]
                        constraints += [psi_t_l >= w1[0,t]*(1-self.gamma1)**t * psi_0_0_l]
                        
                        if  t <15 and t > 0:
                            cost += cvxpy.quad_form(([1,0]-w2[:,t]), self.Pw2)
                            cost += cvxpy.quad_form(slack_hocbf2[:,t], self.Rhocbf2)
                            at_l_1,bt_l_1,ct_l_1 = tangent_to_ellipse(self.a_l*self.vehicle_length, self.b_l*self.vehicle_width, prediction_ahead[0:2,t], [oS_esti[t],oey_esti[t]])
                            psi_0_tplus1_l = at_l_1 * x[3,t+1] + bt_l_1 * x[4,t+1] + ct_l_1
                            constraints += [psi_0_tplus1_l + (self.gamma1-1)*psi_t_l - (1-self.gamma2)**t * psi_0_1_l + slack_hocbf2[0,t]>= w2[0,t]*(self.gamma1-1)*(1-self.gamma2)**t*psi_0_0_l]
                            constraints += [psi_0_tplus1_l + (self.gamma1-1)*psi_t_l - (1-self.gamma2)**t * psi_0_1_l >= w2[0,t]*(self.gamma1-1)*(1-self.gamma2)**t*psi_0_0_l]

                    if prediction_rear is not None and t <= 15:
                        cost += cvxpy.quad_form(([1,0]-w1[:,t]), self.Pw1)
                        cost += cvxpy.quad_form(slackf_hocbf1[:,t], self.Rhocbf1)
                        at_f,bt_f,ct_f = tangent_to_ellipse(self.a_f*self.vehicle_length, self.b_f*self.vehicle_width, prediction_rear[0:2,t], [oS_esti[t],oey_esti[t]])
                        psi_t_f = at_f * x[3,t] + bt_f * x[4,t] + ct_f
                        constraints += [psi_t_f + slackf_hocbf1[0,t]>= w1[0,t]*(1-self.gamma1)**t * psi_0_0_f]
                        constraints += [psi_t_f >= w1[0,t]*(1-self.gamma1)**t * psi_0_0_f]
                        
                        if t < 15 and t > 0:
                            cost += cvxpy.quad_form(([1,0]-w2[:,t]), self.Pw2)
                            cost += cvxpy.quad_form(slackf_hocbf2[:,t], self.Rhocbf2)
                            at_f_1,bt_f_1,ct_f_1 = tangent_to_ellipse(self.a_f*self.vehicle_length, self.b_f*self.vehicle_width, prediction_rear[0:2,t], [oS_esti[t],oey_esti[t]])
                            psi_0_tplus1_f = at_f_1 * x[3,t+1] + bt_f_1 * x[4,t+1] + ct_f_1
                            constraints += [psi_0_tplus1_f + (self.gamma1-1)*psi_t_f - (1-self.gamma2)**t * psi_0_1_f + slackf_hocbf2[0,t]>= w2[0,t]*(self.gamma1-1)*(1-self.gamma2)**t*psi_0_0_f]
                            constraints += [psi_0_tplus1_f + (self.gamma1-1)*psi_t_f - (1-self.gamma2)**t * psi_0_1_f >= w2[0,t]*(self.gamma1-1)*(1-self.gamma2)**t*psi_0_0_f]
                    
                    if prediction_sl_target is not None:
                        cost += cvxpy.quad_form(slack_cbf_tl[:, t], self.Rs)
                        ht_t = prediction_sl_target[t]-x[3,t]-0.5*x[0,t]-5.0-self.vehicle_length
                        htplus1_t = prediction_sl_target[t+1]-x[3,t+1]-0.5*x[0,t+1]-5.0-self.vehicle_length
                        Delta_h_t = htplus1_t - ht_t
                        constraints += [Delta_h_t + self.gamma_r*ht_t + slack_cbf_tl[0,t]>= 0]

                if C_label_additive == "Probe":
                    if prediction_ahead is not None and t <=15:
                        cost += cvxpy.quad_form(([1,0]-w1[:,t]), self.Pw1)
                        cost += cvxpy.quad_form(slack_hocbf1[:,t], self.Rhocbf1)
                        at_l,bt_l,ct_l = tangent_to_ellipse(self.a_l*self.vehicle_length, self.b_l*self.vehicle_width, prediction_ahead[0:2,t], [oS_esti[t],oey_esti[t]])
                        psi_t_l = at_l * x[3,t] + bt_l * x[4,t] + ct_l
                        constraints += [psi_t_l + slack_hocbf1[0,t]>= w1[0,t]*(1-self.gamma1)**t * psi_0_0_l]
                        constraints += [psi_t_l >= w1[0,t]*(1-self.gamma1)**t * psi_0_0_l]
                        
                        if t < 15 and t > 0:
                            cost += cvxpy.quad_form(([1,0]-w2[:,t]), self.Pw2)
                            cost += cvxpy.quad_form(slack_hocbf2[:,t], self.Rhocbf2)
                            at_l_1,bt_l_1,ct_l_1 = tangent_to_ellipse(self.a_l*self.vehicle_length, self.b_l*self.vehicle_width, prediction_ahead[0:2,t], [oS_esti[t],oey_esti[t]])
                            psi_0_tplus1_l = at_l_1 * x[3,t+1] + bt_l_1 * x[4,t+1] + ct_l_1
                            constraints += [psi_0_tplus1_l + (self.gamma1-1)*psi_t_l - (1-self.gamma2)**t * psi_0_1_l + slack_hocbf2[0,t]>= w2[0,t]*(self.gamma1-1)*(1-self.gamma2)**t*psi_0_0_l]
                            constraints += [psi_0_tplus1_l + (self.gamma1-1)*psi_t_l - (1-self.gamma2)**t * psi_0_1_l >= w2[0,t]*(self.gamma1-1)*(1-self.gamma2)**t*psi_0_0_l]
                                        
            # longitudinal constraints in ego lane
            if C_label_additive == "No Probe":
                if prediction_sl_ego is not None:
                    cost += cvxpy.quad_form(slack_cbf[:, t], self.Rs)
                    ht = prediction_sl_ego[t]-x[3,t]-second_paraml[t]*x[0,t]-third_paraml[t]-self.vehicle_length
                    htplus1 = prediction_sl_ego[t+1]-x[3,t+1]-second_paraml[t+1]*x[0,t+1]-third_paraml[t+1]-self.vehicle_length
                    Delta_h = htplus1 - ht
                    constraints += [Delta_h + self.gamma_r*ht + slack_cbf[0,t]>= 0]

            # surround constraints in other lanes
            if path_now == 0:
                dR_min_surround = surround_constraints
                if dR_min_surround is not None:
                    cost += cvxpy.quad_form(slack_surroundr[:,t], self.Rssr)
                    constraints += [x[4,t] + slack_surroundr[0,t] >= dR_min_surround[t]]

            elif path_now == 1:
                dL_min_surround,dR_min_surround = surround_constraints
                if dL_min_surround is not None:
                    cost += cvxpy.quad_form(slack_surroundl[:,t], self.Rssl)
                    constraints += [x[4,t] + slack_surroundl[0,t]<= dL_min_surround[t]]
     
                if dR_min_surround is not None:
                    cost += cvxpy.quad_form(slack_surroundr[:,t], self.Rssr)
                    constraints += [x[4,t] + slack_surroundr[0,t]>= dR_min_surround[t]]
  
            elif path_now == 2:
                dL_min_surround = surround_constraints
                if dL_min_surround is not None:
                    cost += cvxpy.quad_form(slack_surroundl[:,t], self.Rssl)
                    constraints += [x[4,t] + slack_surroundl[0,t]<= dL_min_surround[t]]

            # constraints for lane restriction
            if C_label_virtual == "K":
                constraints += [x[4,t] <= 0.01 + d[0,t]]
                constraints += [x[4,t] + d[1,t] >= -0.01]
            #    constraints += [d[:,t] >= 0]
                
            elif C_label_virtual == "L":
                constraints += [x[4,t] <= -3.45 + d[0,t]]
            #    constraints += [d[:,t] >= 0]
                
            elif  C_label_virtual == "R":
                constraints += [x[4,t] + d[0,t] >= 3.45]
            #    constraints += [d[:,t] >= 0]
 
            A, B, C = ugv.linearized_discretization(xbar[:,t], dref[:,t], k_profile[0,t], self.dt)
            
            if self.add_residual:
                residual_vy,residual_w = self.get_GPR_residual(xbar,dref,GPR_vy,GPR_w)
                constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C + residual_vy[:,t] + residual_w[:,t]]
            else:
                constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

            if t < (self.T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], self.Rd)
                constraints += [u[1, t + 1] - u[1, t]>= self.MIN_DSTEER * self.DU_TH]
                constraints += [u[0, t + 1] - u[0, t] <= self.MAX_DACCEL * self.DU_TH]
                constraints += [u[0, t + 1] - u[0, t] >= self.MIN_DACCEL * self.DU_TH]
        
        cost += cvxpy.quad_form(d[:,self.T-1], self.P)
        cost += cvxpy.quad_form(reference[:, self.T-1] - x[:, self.T], self.Q[self.T-1])
        cost += cvxpy.quad_form(x[4:, self.T], self.Qt)
        
        # surround constraints in other lanes
        if path_now == 0:
            dR_min_surround = surround_constraints
            if dR_min_surround is not None:
                cost += cvxpy.quad_form(slack_surroundr[:,self.T-1], self.Rssr)
                constraints += [x[4,self.T] + slack_surroundr[0,self.T-1] >= dR_min_surround[self.T]]

        elif path_now == 1:
            dL_min_surround,dR_min_surround = surround_constraints
            if dL_min_surround is not None:
                cost += cvxpy.quad_form(slack_surroundl[:,self.T-1], self.Rssl)
                constraints += [x[4,self.T] + slack_surroundl[1,self.T-1]<= dL_min_surround[self.T]]
                
            if dR_min_surround is not None:
                cost += cvxpy.quad_form(slack_surroundr[:,self.T-1], self.Rssr)
                constraints += [x[4,self.T] + slack_surroundr[0,self.T-1]>= dR_min_surround[self.T]]
                
        elif path_now == 2:
            dL_min_surround = surround_constraints
            if dL_min_surround is not None:
                cost += cvxpy.quad_form(slack_surroundl[:,self.T-1], self.Rssl)
                constraints += [x[4,self.T] + slack_surroundl[0,self.T-1]<= dL_min_surround[self.T]]

        # constraints for lane restriction
        if  C_label_virtual == "K":
            constraints += [x[4,self.T] <= 0.01 + d[0,self.T-1]]
            constraints += [x[4,self.T] + d[1,self.T-1] >= -0.01]
            # constraints += [d[:,self.T-1] >= 0]
            
        elif C_label_virtual == "L":
            constraints += [x[4,self.T]  <= -3.45 + d[0,self.T-1]]
            # constraints += [d[:,self.T-1] >= 0]
            
        elif C_label_virtual == "R":
            constraints += [x[4,self.T] + d[0,self.T-1] >= 3.45]
            # constraints += [d[:,self.T-1] >= 0]
     
        constraints += [x[:, 0] == x0[:]]
        constraints += [u[0, :] <= self.MAX_ACCEL ]
        constraints += [u[0, :] >= self.MIN_ACCEL]
        # constraints += [slack_acc[:,:] >=0]
        # constraints += [slack_delta[:,:] >=0]
        constraints += [cvxpy.abs(u[1, :]) <= self.MAX_STEER]
        # constraints += [x[5,:] >= -np.pi]
        # constraints += [x[5,:] <= np.pi]
        
        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.ECOS, verbose=False)
        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            ovx = get_nparray_from_matrix(x.value[0, :])
            ovy = get_nparray_from_matrix(x.value[1, :])
            owz = get_nparray_from_matrix(x.value[2, :])
            os = get_nparray_from_matrix(x.value[3, :])
            oey = get_nparray_from_matrix(x.value[4, :])
            oepsi = get_nparray_from_matrix(x.value[5, :])
            oa = get_nparray_from_matrix(u.value[0, :])
            odelta = get_nparray_from_matrix(u.value[1, :])
        else:
            print("Error: Cannot solve mpc..")
            ovx, ovy, owz, os, oey, oepsi, oa, odelta   = None, None, None, None, None, None, None, None
        return oa, odelta, ovx, ovy, owz, os, oey, oepsi