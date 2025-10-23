import numpy as np

def params():     
    mass= 1292
    lf= 1.56
    lr= 1.04
    Iz= 1343.1
    Df= 0.8 * 1270 * 9.81 / 2.0
    Cf= 1.25
    Bf= 1.0
    Dr= 0.8 * 1270 * 9.81 / 2.0
    Cr= 1.25
    Br= 1.0
    
    params = {
        'lf': lf,
        'lr': lr,
        'mass': mass,
        'Iz': Iz,
        'Cf': Cf,
        'Cr': Cr,
        'Df': Df,
        'Dr': Dr,
        'Bf': Bf,
        'Br': Br
        }
    
    return params

# def params_for_matlab():
#     mass= 1292
#     lf= 1.56
#     lr= 1.04
#     Iz= 1343.1
#     Df= 0.8 * 1270 * 9.81 / 2.0
#     Cf= 1.25
#     Bf= 1.0
#     Dr= 0.8 * 1270 * 9.81 / 2.0
#     Cr= 1.25
#     Br= 1.0
    
#     return mass, lf, lr, Iz, Df, Cf, Bf, Dr, Cr, Br