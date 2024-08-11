
def sodm_params():     
    Bc = 15.0
    Bc_l = 8.0
    d0 = 10.0
    alpha_l = -0.8
    rx = 10.0
    H_pred = 5
    dt = 0.1
    desired_v = 18.0

    params = {
        "Bc":Bc,
        "Bc_l":Bc_l,
        "d0":d0,
        "alpha_l":alpha_l,
        "rx":rx,
        "H_pred":H_pred,
        "dt":dt,
        "desired_v":desired_v
    }   
    return params