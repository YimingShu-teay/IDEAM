import numpy as np

def surrounding_params():     
    a_max = 3.0
    delta = 4
    s0 = 2
    b = 1.7
    T = 1.6
    K_P = 0.5
    K_I = 5
    K_D = 5
    dt = 0.1
    lf = 1.56
    lr = 1.04
    length = 3.5
    
    params = {
        'a_max': a_max,
        'delta': delta,
        's0': s0,
        'b': b,
        'T': T,
        'K_P': K_P,
        'K_I': K_I,
        'K_D': K_D,
        'dt': dt,
        'lf': lf,
        'lr': lr,
        'length':length
        }
    
    return params