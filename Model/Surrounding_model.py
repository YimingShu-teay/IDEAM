import numpy as np
from collections import deque

class Curved_Road_Vehicle:
    def __init__(self, a_max, delta, s0, b, T, K_P, K_D, K_I, dt, lf, lr,length, **kwargs):
        self.a_max = a_max
        self.delta = delta
        self.s0 = s0
        self.b = b
        self.T = T
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        self.dt = dt
        self.e_buffer = deque(maxlen=10)
        self.lf = lf
        self.lr = lr
        self.length = length
        
        
    def Longitudinal_IDM_controller(self, s, v, vd, s_ahead, v_ahead):
        if v_ahead is None:
            a = self.a_max * (1 - (v / vd)**self.delta)
            
        else:
            Dv = v - v_ahead
            Ds = s_ahead - s - self.length
            s_star = max(self.s0 + self.length, self.s0 + self.length + self.T*v + v*Dv/(2*np.sqrt(self.a_max*self.b)))
            a = self.a_max*(1 - (v/vd)**self.delta - (s_star/Ds)**2)
        return a
    
    
    def Lateral_PID_controller(self, s, v, a, path, steer_range):
        v_next = v + a*self.dt
        s_d = s + v_next*self.dt
        x, y = path(s)
        x_d, y_d = path(s_d)
        yaw = path.get_theta_r(s)
        yaw_d = path.get_theta_r(s_d)
        
        d_yaw = yaw_d - yaw

        X_vec = [np.cos(yaw), np.sin(yaw), 0.0]
        X_d_vec = [x_d - x, y_d - y, 0.0]
        cross = np.cross(X_vec, X_d_vec)
    
        if cross[2] < 0:
            d_yaw *= -1.0
            
        self.e_buffer.append(d_yaw)
            
        if len(self.e_buffer) >= 2:
            de = (self.e_buffer[-1] - self.e_buffer[-2]) / self.dt
            ie = sum(self.e_buffer) * self.dt
        else:
            de = 0.0
            ie = 0.0
        steer_angle = (self._K_P * d_yaw) + (self._K_D * de /
                                             self.dt) + (self._K_I * ie * self.dt)
        
        if steer_angle > steer_range[1]:
            steer_angle = steer_range[1]
            
        if steer_angle < steer_range[0]:
            steer_angle = steer_range[0]            
        
        return steer_angle
    
    
    def update_states(self, s, v, vd, s_ahead, v_ahead, path, steer_range):
        a = self.Longitudinal_IDM_controller(s, v, vd, s_ahead, v_ahead)
        steer_angle = self.Lateral_PID_controller(s, v, a, path, steer_range)
        
        x, y = path(s)
        yaw = path.get_theta_r(s)
   
        beta = np.arctan2(self.lr* np.tan(steer_angle),(self.lf + self.lr));
        dx =  v * np.cos(yaw + beta) * self.dt
        dy = v * np.sin(yaw + beta) * self.dt
        dyaw = (v * np.sin(beta) / self.lr) * self.dt
        dv = a * self.dt
        
        x_next = x + dx
        y_next = y + dy
        yaw_next = yaw + dyaw
        v_next = v + dv
        
        return x_next, y_next, yaw_next, v_next, steer_angle, a
    
    

        
        