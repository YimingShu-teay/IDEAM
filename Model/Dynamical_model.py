import sys
sys.path.append(r"C:\Users\sym02\Desktop\Research\Extension\codes\copy_test") 
import numpy as np
# import casadi as cs
from Model.model import *
import Control.utils as util
import math


class Dynamic(Model):

    def __init__(self, lf, lr, mass, Iz, Cf, Cr, 
                 Bf=None, Br=None, Df=None, Dr=None, **kwargs):
        """	specify model params here
        """
        self.lf = lf
        self.lr = lr
        self.dr = lr/(lf+lr)
        self.mass = mass
        self.Iz = Iz

        self.Cf = Cf
        self.Cr = Cr

        self.Bf = Bf
        self.Br = Br
        self.Df = Df
        self.Dr = Dr
        
        self.n_states = 6
        self.n_inputs = 2
        Model.__init__(self)


    def _diffequation(self, t, x, u, k):
        """	write dynamics as first order ODE: dxdt = f(x(t))
            x is a 6x1 vector: [x, y, psi, vx, vy, omega]^T
            u is a 2x1 vector: [acc/pwm, steer]^T
        """
        steer = u[1]
        vx = x[0]
        vy = x[1]
        w = x[2]
        ey = x[4]
        epsi = x[5]

        Ffy, Frx, Fry = self.calc_forces(x, u)

        dxdt = np.zeros(6)
        dxdt[0] = 1/self.mass * (Frx - Ffy*np.sin(steer)) + vy*w
        dxdt[1] = 1/self.mass * (Fry + Ffy*np.cos(steer)) - vx*w
        dxdt[2] = 1/self.Iz * (Ffy*self.lf*np.cos(steer) - Fry*self.lr)
        dxdt[3] = (vx*np.cos(epsi) - vy*np.sin(epsi)) / (1 - k*ey)
        dxdt[4] = vx*np.sin(epsi) + vy*np.cos(epsi)
        dxdt[5] = w - (vx*np.cos(epsi) - vy*np.sin(epsi)) / (1 - k*ey)*k
        return dxdt


    def calc_forces(self, x, u, return_slip=False):
        """	PACEJKA TIRE MODEL
        """
        acc = u[0]
        steer = u[1]
        vx = x[0]
        vy = x[1]
        w = x[2]
 
        Frx = self.mass*acc

        alphaf = steer - np.arctan2((self.lf*w + vy), abs(vx))
        alphar = np.arctan2((self.lr*w - vy), abs(vx))
        Ffy = 2*self.Df * np.sin(self.Cf * np.arctan(self.Bf * alphaf))
        Fry = 2*self.Dr * np.sin(self.Cr * np.arctan(self.Br * alphar))
        
        if return_slip:
            return Ffy, Frx, Fry, alphaf, alphar
        else:
            return Ffy, Frx, Fry


    # def casadi(self, x, u, k, dxdt):
    #     """	write dynamics as first order ODE: dxdt = f(x(t))
    #         x is a 6x1 vector: [x, y, psi, vx, vy, omega]^T
    #         u is a 2x1 vector: [acc/pwm, steer]^T
    #         dxdt is a casadi.SX variable
    #     """
    #     a = u[0]
    #     steer = u[1]
    #     vx = x[0]
    #     vy = x[1]
    #     w = x[2]
    #     ey = x[4]
    #     epsi = x[5]

    #     vmin = 0.05
    #     vy = cs.if_else(vx<vmin, 0, vy)
    #     w = cs.if_else(vx<vmin, 0, w)
    #     steer = cs.if_else(vx<vmin, 0, steer)
    #     vx = cs.if_else(vx<vmin, vmin, vx)

    #     Frx = self.mass*a
    #     alphaf = steer - cs.atan2((self.lf*w + vy), vx)
    #     alphar = cs.atan2((self.lr*w - vy), vx)
    #     Ffy = self.Df * cs.sin(self.Cf * cs.arctan(self.Bf * alphaf))
    #     Fry = self.Dr * cs.sin(self.Cr * cs.arctan(self.Br * alphar))

    #     dxdt[0] = 1/self.mass * (Frx - Ffy*np.sin(steer)) + vy*w
    #     dxdt[1] = 1/self.mass * (Fry + Ffy*np.cos(steer)) - vx*w
    #     dxdt[2] = 1/self.Iz * (Ffy*self.lf*np.cos(steer) - Fry*self.lr)
    #     dxdt[3] = (vx*np.cos(epsi) - vy*np.sin(epsi)) / (1 - k*ey)
    #     dxdt[4] = vx*np.sin(epsi) + vy*np.cos(epsi)
    #     dxdt[5] = w - (vx*np.cos(epsi) - vy*np.sin(epsi)) / (1 - k*ey)*k
        
    #     return dxdt


    def linearized_discretization(self, x0, u0, k, dt):
        """	linearize at a given x0, u0
            for a given continuous system dxdt = f(x(t))
            calculate A = ∂f/∂x, B = ∂f/∂u, g = f evaluated at x0, u0
            A is 6x6, B is 6x2, g is 6x1
        """
        a = u0[0]
        steer = u0[1]
        vx = x0[0]
        vy = x0[1]
        w = x0[2]
        ey = x0[4]
        epsi = x0[5]
        epsi = util.normalize_angle(epsi)
        
        steer_max = math.radians(25.0)
        steer_min = math.radians(-25.0)
        steer = max(min(steer, steer_max), steer_min)
        
        sin_delta = np.sin(steer)
        cos_delta = np.cos(steer)
        sin_epsi = np.sin(epsi)
        cos_epsi = np.cos(epsi)

        Ffy, Frx, Fry, alphaf, alphar = self.calc_forces(x0, u0, return_slip=True)

        dFfy_dalphaf = self.Bf * self.Cf * self.Df * np.cos(self.Cf * np.arctan(self.Bf * alphaf))
        dFfy_dalphaf *= 1/(1+(self.Bf*alphaf)**2)

        dFry_dalphar = self.Br * self.Cr * self.Dr * np.cos(self.Cr * np.arctan(self.Br * alphar))
        dFry_dalphar *= 1/(1+(self.Br*alphar)**2)
        
        dFfy_delta = dFfy_dalphaf

        dFfy_dvx =  dFfy_dalphaf * (self.lf*w + vy)/((self.lf*w + vy)**2 + vx**2)
        dFfy_dvy = -dFfy_dalphaf * vx/((self.lf*w + vy)**2 + vx**2)
        dFfy_dw = -dFfy_dalphaf * self.lf * vx/((self.lf*w + vy)**2 + vx**2)

        dFry_dvx = -dFry_dalphar * (self.lr*w - vy)/((self.lr*w - vy)**2 + vx**2)
        dFry_dvy = -dFry_dalphar * vx/((self.lr*w - vy)**2 + vx**2)
        dFry_dw = dFry_dalphar * self.lr * vx/((self.lr*w - vy)**2 + vx**2)

        f1_vx = -dFfy_dvx*sin_delta #此处有所改动
        f1_vy = 1/self.mass * (-dFfy_dvy*sin_delta + self.mass*w)
        f1_w = 1/self.mass * (-dFfy_dw*sin_delta + self.mass*vy)

        f2_vx = 1/self.mass * (dFry_dvx + dFfy_dvx*cos_delta - self.mass*w)
        f2_vy = 1/self.mass * (dFry_dvy + dFfy_dvy*cos_delta)
        f2_w = 1/self.mass * (dFry_dw + dFfy_dw*cos_delta - self.mass*vx)

        f3_vx = 1/self.Iz * (dFfy_dvx*self.lf*cos_delta - dFry_dvx*self.lr)
        f3_vy = 1/self.Iz * (dFfy_dvy*self.lf*cos_delta - dFry_dvy*self.lr)
        f3_w = 1/self.Iz * (dFfy_dw*self.lf*cos_delta - dFry_dw*self.lr)
        
        f4_vx = cos_epsi/(1 - k*ey)
        f4_vy = -sin_epsi/(1 - k*ey)
        f4_ey = (vx*cos_epsi - vy*sin_epsi)/((1 - k*ey)**2) * (-k)
        f4_epsi =  (-vx*sin_epsi - vy*cos_epsi)/(1 - k*ey)
        
        f5_vx = sin_epsi
        f5_vy = cos_epsi
        f5_epsi = (vx*cos_epsi - vy*sin_epsi)
        
        f6_vx =  -cos_epsi/(1 - k*ey)*k
        f6_vy = sin_epsi/(1 - k*ey)*k
        f6_w = 1
        f6_ey = (vx*cos_epsi - vy*sin_epsi) / ((1 - k*ey)**2)*(k**2)
        f6_epsi = -(-vx*sin_epsi - vy*cos_epsi) /(1 - k*ey)*k
    
        f1_u1 = 1
        f1_delta = 1/self.mass * (-dFfy_delta*sin_delta - Ffy*cos_delta)
        f2_delta = 1/self.mass * (dFfy_delta*cos_delta - Ffy*sin_delta)
        f3_delta = 1/self.Iz * (dFfy_delta*self.lf*cos_delta - Ffy*self.lf*sin_delta)
        

        A_ = np.array([
            [f1_vx, f1_vy, f1_w, 0, 0, 0],
            [f2_vx, f2_vy, f2_w, 0, 0, 0],
            [f3_vx, f3_vy, f3_w, 0, 0, 0],
            [f4_vx, f4_vy, 0, 0, f4_ey, f4_epsi],
            [f5_vx, f5_vy, 0, 0, 0, f5_epsi],
            [f6_vx, f6_vy, f6_w, 0, f6_ey, f6_epsi],
            ])
        B_ = np.array([
            [f1_u1, f1_delta],
            [0, f2_delta],
            [0, f3_delta],
            [0, 0],
            [0, 0],
            [0, 0],
            ])
        
        g = self._diffequation(None, x0, u0, k).reshape(-1,)
        
        I = np.eye(6)
        
        A = dt*A_ + I
        B = dt*B_
        C = dt*(g-np.dot(A_,x0)-np.dot(B_,u0))
         
        return A, B, C
    
    def propagate(self, x0, u0, dt, x0_g, path_d, sample, x_list, y_list, bound):
        
        x_next = np.zeros(6) 
        x_next_g = np.zeros(3)
        a_lon_lat = np.zeros(2)
        
        Ffy, Frx, Fry = self.calc_forces(x0, u0)
        
        a = u0[0]
        steer = u0[1]
        vx = x0[0]
        vy = x0[1]
        w = x0[2]
        s = x0[3]
        ey = x0[4]
        epsi = x0[5]
        epsi = util.normalize_angle(epsi)
        
        k = path_d.get_k(s)
        ds = (vx*np.cos(epsi) - vy*np.sin(epsi))/(1-k*ey)
        
        
        X = x0_g[0]
        Y = x0_g[1]
        psi = x0_g[2]
        
        psi = util.normalize_angle(psi)
        
        a_lon_lat[0] = (a - 1/self.mass*Ffy*np.sin(steer) + w*vy)
        a_lon_lat[1] = (1 / self.mass*(Ffy*np.cos(steer) + Fry) - w*vx)

        x_next[0] = vx + dt*(a - 1/self.mass*Ffy*np.sin(steer) + w*vy)
        x_next[1] = vy + dt*(1 / self.mass*(Ffy*np.cos(steer) + Fry) - w*vx)
        x_next[2] = w + dt *(1 / self.Iz*(self.lf*Ffy*np.cos(steer) - self.lr*Fry))
        
        # x_next[3] = s + dt*((vx*np.cos(epsi) - vy*np.sin(epsi))/(1 - k*ey))
        # x_next[4] = ey + dt*(vx*np.sin(epsi) + vy*np.cos(epsi))
        # x_next[5] = epsi + dt*(w - (vx*np.cos(epsi) - vy*np.sin(epsi))/(1 - k*ey)*k)
        
        # disturbance_x,disturbance_y = util.get_disturbance(x0_g[2],x0[0],x0[1])
        x_next_g[0] = X + dt*(((vx)*np.cos(psi) - (vy)*np.sin(psi))) 
        x_next_g[1] = Y + dt * ((vx)*np.sin(psi) + (vy)*np.cos(psi)) 
        x_next_g[2] = psi + dt*(w)
        
        x_next_g[2] = util.normalize_angle(x_next_g[2])
        
        s_next, ey_next, epsi_next = util.find_frenet_coord(path_d,x_list,y_list,sample,x_next_g)
        
        epsi_next = util.normalize_angle(epsi_next)
        
        x_next[3] = s_next
        x_next[4] = ey_next
        x_next[5] = epsi_next
        
        return x_next, x_next_g, a_lon_lat, ds

    def propagate_iter(self, x0, u0, k, dt, x0_g):
        
        x_next = np.zeros(6) 
        x_next_g = np.zeros(3)
        
        Ffy, Frx, Fry = self.calc_forces(x0, u0)
        
        a = u0[0]
        steer = u0[1]
        vx = x0[0]
        vy = x0[1]
        w = x0[2]
        s = x0[3]
        ey = x0[4]
        epsi = x0[5]
        epsi = util.normalize_angle(epsi)
        
        X = x0_g[0]
        Y = x0_g[1]
        psi = x0_g[2]
        
        psi = util.normalize_angle(psi)

        x_next[0] = vx + dt*(a - 1/self.mass*Ffy*np.sin(steer) + w*vy)
        x_next[1] = vy + dt*(1 / self.mass*(Ffy*np.cos(steer) + Fry) - w*vx)
        x_next[2] = w + dt *(1 / self.Iz*(self.lf*Ffy*np.cos(steer) - self.lr*Fry))
        
        x_next[3] = s + dt*((vx*np.cos(epsi) - vy*np.sin(epsi))/(1 - k*ey))
        x_next[4] = ey + dt*(vx*np.sin(epsi) + vy*np.cos(epsi))
        x_next[5] = epsi + dt*(w - (vx*np.cos(epsi) - vy*np.sin(epsi))/(1 - k*ey)*k)
        
        # disturbance_x,disturbance_y = util.get_disturbance(x0_g[2],x0[0],x0[1])
        x_next_g[0] = X + dt*(((vx)*np.cos(psi) - (vy)*np.sin(psi))) 
        x_next_g[1] = Y + dt * ((vx)*np.sin(psi) + (vy)*np.cos(psi)) 
        x_next_g[2] = psi + dt*(w)
        
        x_next_g[2] = util.normalize_angle(x_next_g[2])
        
        return x_next, x_next_g