import math
def mobil_params():     
    v_0 = 18.0
    s_0 = 2.0
    a = 1.5
    b = 2.0
    b_safe = 4.0
    delta = 4
    politeness = 0.3
    T = 3.0
    change_threshold = 0.1


    params = {
        "v_0":v_0,
        "s_0":s_0,
        "a":a,
        "b":b,
        "b_safe":b_safe,
        "delta":delta,
        "politeness":politeness,
        "T":T,
        "change_threshold":change_threshold
    }   
    return params