import math
import numpy as np

def constraint_params():     
    NX = 6
    NU = 2
    T = 30
    TARGET_SPEED = 30.0
    MAX_ACCEL = 3.0
    MIN_ACCEL = -3.0
    MAX_SPEED = 33.3
    MIN_SPEED = 2.0
    MAX_STEER = math.radians(25.0)
    MIN_STEER = math.radians(-25.0)
    MAX_DACCEL = 3.0
    MIN_DACCEL = -6.0
    MAX_DSTEER = math.radians(15.0)
    MIN_DSTEER = math.radians(-15.0)
    R = np.diag([1.0,80.0])*0.1
    Rd = np.diag([0.0,8.0])*0.1
    Q_compare = np.diag([2.0,6.0,10.0,0.0,100.0,100.0])
    Q = [np.diag([2.0,40.0,40.0,0.0,60.0,70.0])*0.1,
          np.diag([2.0,40.0,40.0,0.0,60.0,70.0])*0.1,
          np.diag([2.0,40.0,40.0,0.0,60.0,70.0])*0.1,
          np.diag([2.0,40.0,40.0,0.0,60.0,80.0])*0.1,
          np.diag([2.0,40.0,40.0,0.0,60.0,80.0])*0.1,
          np.diag([2.0,40.0,40.0,0.0,70.0,80.0])*0.1,
          np.diag([2.5,40.0,40.0,0.0,70.0,80.0])*0.1,
          np.diag([2.5,40.0,40.0,0.0,70.0,80.0])*0.1,
          np.diag([2.5,40.0,40.0,0.0,70.0,90.0])*0.1,
          np.diag([2.5,40.0,40.0,0.0,70.0,90.0])*0.1,
          np.diag([3.0,40.0,40.0,0.0,80.0,90.0])*0.1,
          np.diag([3.0,40.0,40.0,0.0,80.0,90.0])*0.1,
          np.diag([3.0,40.0,40.0,0.0,90.0,90.0])*0.1,
          np.diag([3.0,40.0,40.0,0.0,90.0,90.0])*0.1,
          np.diag([3.0,40.0,40.0,0.0,90.0,90.0])*0.1,
          np.diag([3.0,40.0,40.0,0.0,90.0,100.0])*0.1,
          np.diag([3.0,40.0,40.0,0.0,90.0,100.0])*0.1,
          np.diag([3.0,40.0,40.0,0.0,90.0,100.0])*0.1,
          np.diag([3.0,40.0,40.0,0.0,90.0,110.0])*0.1,
          np.diag([3.0,40.0,40.0,0.0,100.0,120.0])*0.1,
          np.diag([3.5,40.0,40.0,0.0,100.0,120.0])*0.1,
          np.diag([3.5,40.0,40.0,0.0,100.0,120.0])*0.1,
          np.diag([3.5,40.0,40.0,0.0,100.0,120.0])*0.1,
          np.diag([3.5,40.0,40.0,0.0,100.0,120.0])*0.1,
          np.diag([3.5,40.0,40.0,0.0,100.0,140.0])*0.1,
          np.diag([3.5,40.0,40.0,0.0,100.0,160.0])*0.1,
          np.diag([3.5,40.0,40.0,0.0,100.0,180.0])*0.1,
          np.diag([3.5,40.0,40.0,0.0,100.0,200.0])*0.1,
          np.diag([3.5,40.0,40.0,0.0,100.0,240.0])*0.1,
          np.diag([3.5,40.0,40.0,0.0,100.0,240.0])*0.1]
    Qt = np.diag([0.0,600.0])*0.1
    P = np.diag([5.0,0.0])*0.1
    Pw1 = np.diag([5.0,0.0])*0.1 #这里修一个mpc重解，还有一个决策问题需要修复
    Pw2 = np.diag([5.0,0.0])*0.1
    Pw3 = np.diag([10,0.0])*0.1
    Pw4 = np.diag([10,0.0])*0.1
    Rs =  np.diag([10,0.0])*0.1
    Rs_f = np.diag([10,0.0])*0.1
    Rs_tl =np.diag([10,0.0])*0.1
    Rss =  np.diag([900,0.0])*0.1
    Rssl = np.diag([600,0])*0.1
    Rssr = np.diag([600,0])*0.1
    Rhocbf1 = np.diag([600,0])*0.1
    Rhocbf2 = np.diag([600,0])*0.1
    slack_a = np.diag([6000,6000])*0.1
    slack_d = np.diag([5000,0])*0.1
    
    Lane_width = 3.5
    vehicle_width = 1.2
    vehicle_length = 3.5
    MAX_ITER = 1
    DU_TH = 0.1
    
    gamma1 = 0.8
    gamma2 = 0.8
    gamma_r = 0.9
    a_l = 1.5
    b_l = 2.2
    a_f = 2.0
    b_f = 2.5
    dt = 0.1
    
    params = {
        'NX': NX,
        'NU': NU,
        'T': T,
        'TARGET_SPEED': TARGET_SPEED,
        'MAX_ACCEL': MAX_ACCEL,
        'MIN_ACCEL': MIN_ACCEL,
        'MAX_SPEED': MAX_SPEED,
        'MIN_SPEED': MIN_SPEED,
        'MAX_STEER': MAX_STEER,
        'MIN_STEER': MIN_STEER,
        'MAX_DACCEL': MAX_DACCEL,
        'MIN_DACCEL': MIN_DACCEL,
        'MAX_DSTEER': MAX_DSTEER,
        'MIN_DSTEER': MIN_DSTEER,
        'R' : R,
        'Rd' : Rd,
        'Q_compare':Q_compare,
        'Q' : Q,
        'Qt' :Qt,
        'P' : P,
        'Pw1': Pw1,
        'Pw2':Pw2,
        'Pw3':Pw3,
        'Pw4':Pw4,
        "Rs":Rs,
        'Rs_f':Rs_f,
        "Rs_tl":Rs_tl,
        "Rss":Rss,
        "Rssl":Rssl,
        "Rssr":Rssr,
        "Rhocbf1":Rhocbf1,
        "Rhocbf2":Rhocbf2,
        "slack_a":slack_a,
        "slack_d":slack_d,
        'Lane_width' : Lane_width,
        'vehicle_width': vehicle_width,
        'vehicle_length':vehicle_length,
        'MAX_ITER':MAX_ITER,
        'DU_TH': DU_TH,
        'gamma1': gamma1,
        'gamma2': gamma2,
        'gamma_r':gamma_r,
        'a_l': a_l,
        'b_l': b_l,
        'a_f':a_f,
        'b_f':b_f,
        'dt': dt
    }   
    
    return params


# def constraint_params_for_matlab():     
#     NX = 6
#     NU = 2
#     T = 20
#     TARGET_SPEED = 30.0
#     MAX_ACCEL = 3.0
#     MIN_ACCEL = -3.0
#     MAX_SPEED = 33.3
#     MIN_SPEED = 2.0
#     MAX_STEER = math.radians(25.0)
#     MIN_STEER = math.radians(-25.0)
#     MAX_DACCEL = 3.0
#     MIN_DACCEL = -3.0
#     MAX_DSTEER = math.radians(15.0)
#     MIN_DSTEER = math.radians(-15.0)
#     R = np.diag([0,100])
#     Rd = np.diag([0,100])
#     Q = np.diag([2000.0,10.0,100.0,0.0,100.0,0.0])
#     Qt = np.diag([0.0,200.0])
    
#     Lane_width = 3.5
#     vehicle_width = 1.9
    
#     return NX, NU, T, TARGET_SPEED, MAX_ACCEL, MIN_ACCEL, MAX_SPEED, MIN_SPEED, \
#            MAX_STEER, MIN_STEER, MAX_DACCEL, MIN_DACCEL, MAX_DSTEER, MIN_DSTEER,\
#            R, Rd, Q, Qt, Lane_width, vehicle_width