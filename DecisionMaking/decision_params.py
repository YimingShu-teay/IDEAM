import math
def decision_params():     
    rho = 0.01
    a_max_acc_lon = 3.0
    a_max_brake_lon = 6.0
    a_min_brake_lon = 3.0
    l = 3.5
    l_diag = math.sqrt(l**2 + 1.2**2)
    T_risk = 8
    epsilon = 3.5
    k = 3
    threshold = 3.5
    d0 = 5.0
    Td = 0.3

    params = {
        'l_diag':l_diag,
        'l':l,
        "T_risk":T_risk,
        'epsilon':epsilon,
        'k':k,
        'rho': rho,
        'a_max_acc_lon': a_max_acc_lon,
        'a_max_brake_lon': a_max_brake_lon,
        'a_min_brake_lon': a_min_brake_lon,
        'threshold': threshold,
        'd0':d0,
        'Td':Td
    }   
    return params