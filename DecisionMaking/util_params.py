import math
def util_params():     
    rho = 0.01
    a_max_acc_lon = 3.0
    a_max_brake_lon = 6.0
    a_min_brake_lon = 3.0
    vehicle_width = 1.2
    vehicle_length = 3.5
    l = vehicle_length
    l_diag = math.sqrt(l**2 + vehicle_width**2)
    mu = 2.2
    T = 70
    dt = 0.1
    lane_width = 3.5
    Th = 0.5
    d0 = 5.0
    vehicle_num = 5

    params = {
        'rho': rho,
        'a_max_acc_lon': a_max_acc_lon,
        'a_max_brake_lon': a_max_brake_lon,
        'a_min_brake_lon': a_min_brake_lon,
        'vehicle_width':vehicle_width,
        'l':l,
        'l_diag':l_diag,
        'mu': mu,
        'T':T,
        'dt':dt,
        'lane_width':lane_width,
        'Th':Th,
        'd0':d0,
        'vehicle_num':vehicle_num
   
    }   
    return params