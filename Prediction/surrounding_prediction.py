import numpy as np

def surrounding_vehicle_prediction(s0,ey0,epsi0,v0,a0,dt,T):
    prediction_list = np.zeros((5,T+1))
    prediction_list[0][0] = s0
    prediction_list[1][0] = ey0
    prediction_list[2][0] = epsi0
    prediction_list[3][0] = v0
    prediction_list[4][0] = a0
    for i in range(T):
        vt = v0 + a0*dt*i
        prediction_list[0,i+1] = prediction_list[0,i] +  vt*dt
        prediction_list[1,i+1] = ey0
        prediction_list[2,i+1] = epsi0
        prediction_list[3,i+1] = vt
        prediction_list[4,i+1] = a0
    return prediction_list

def ego_vehicle_prediction(s0,v0,a0,dt,T):
    prediction_list = np.zeros((4,T+1))
    prediction_list[0][0] = s0
    prediction_list[3][0] = v0
    for i in range(1,T):
        if i == 1:
            vt = v0 + a0*dt*i
        else:
            vt = v0 
    prediction_list[0,i] = prediction_list[0,i] +  vt*dt
    prediction_list[3,i] = vt
    return prediction_list