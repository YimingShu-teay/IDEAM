import sys
sys.path.append("C:\\Users\\sym02\\Desktop\\Research\\Extension\\codes\\decision_improve") 
from Control.MPC import *
from Control.constraint_params import *
from Model.Dynamical_model import *
from Model.params import *
from Model.surrounding_params import *
from Model.Surrounding_model import *
from Model.surrounding_vehicles import *
from Control.HOCBF import *
from DecisionMaking.decision_params import *
from DecisionMaking.give_desired_path import *
from DecisionMaking.util import *
from DecisionMaking.util_params import *
from DecisionMaking.decision import *
from Prediction.surrounding_prediction import *
from progress.bar import Bar
import time
from multiprocessing import Pool


def one_round(round_):
    time_now = int(time.time())
    dirpath = os.path.abspath(os.path.dirname(__file__))

    save_dir = os.path.join(dirpath, "figsave")
    os.makedirs(save_dir, exist_ok=True)

    #给初始时刻赋值
    N_t = 1000
    bar = Bar(max=N_t-1)
    dt = 0.1
    boundary = 1.0

    X0 = [8.0,0.0,0.0,20.0,0.0,0.0]
    X0_g = [path2c(X0[3])[0],path2c(X0[3])[1],path2c.get_theta_r(X0[3])]

    path_center= np.array([path1c,path2c,path3c])
    sample_center = np.array([samples1c, samples2c, samples3c])
    x_center = [x1c, x2c, x3c]
    y_center = [y1c, y2c, y3c]
    x_bound = [x1,x2]
    y_bound = [y1,y2]
    path_bound = [path1,path2]
    path_bound_sample = [samples1,samples2]

    X = [X0_g[0]]
    Y = [X0_g[1]]
    Psi = [X0_g[2]]
    vx = [8.0]
    vy = [0.0]
    w = []
    s = []
    ey = [0.0]
    epsi = [0.0]
    t = [0.0]
    a = [0.0]
    delta = [0.0] 
    oa, od = 0.0, 0.0
    u0 = [oa, od]
    pathRecord = [1]
    path_desired = []

    x_area = 30.0
    y_area = 15.0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          

    steer_range = [math.radians(-8.0),math.radians(8.0)]

    Params = params()
    Constraint_params = constraint_params()
    dynamics = Dynamic(**Params)

    decision_param = decision_params()
    decision_maker = decision(**decision_param)
    mpc_controller = LMPC(**Constraint_params)
    mpc_controller.get_path_curvature(path=path2c)

    params_dir = r"C:\Users\sym02\Desktop\Research\Extension\codes\decision_change_rear\file_save\1718206467.0489624400"
    surroundings = Surrounding_Vehicles(steer_range,dt,boundary)

    util_params_ = util_params()
    utils = LeaderFollower_Uitl(**util_params_)
    mpc_controller.set_util(utils)

    path_changed = 1

    ################## metrics record ##############
    S_obs_record = [] # 与障碍物的最小距离
    initial_params = [surroundings.left_initial_set,surroundings.center_initial_set,surroundings.right_initial_set] #初始时刻的障碍物参数
    initial_vds = [surroundings.vd_left_all,surroundings.vd_center_all,surroundings.vd_right_all]
    progress_20, progress_40 = 0.0, 0.0
    TTC_record = []
    vel = []
    acc = []
    steer_record = []
    path_record = []
    lane_state_record = []
    C_label_record = []


    last_desired_group = None
    for i in range(N_t):
        bar.next()
        ################## surrounding vehicles ###################
        vehicle_left, vehicle_centre, vehicle_right = surroundings.get_vehicles_states()
        
        ########################## path position info ################## 
        path_now = judge_current_position(X0_g[0:2],x_bound,y_bound,path_bound,path_bound_sample)
        path_ego = surroundings.get_path_ego(path_now)
        if path_now == 0:
            start_group_str = "L1"
        elif path_now == 1:
            start_group_str = "C1"
        else:
            start_group_str = "R1"
        
        ############################### decision info ############
        if i == 0:
            ovx, ovy, owz, oS, oey, oepsi = clac_last_X(oa,od,mpc_controller.T,path_ego,dt,6,X0,X0_g)
            last_X = [ovx, ovy, owz, oS, oey, oepsi]
            
        # ego_traj = ego_vehicle_prediction(X0[0],X0[3],u0[0],mpc_controller.dt,mpc_controller.T)
        all_info = utils.get_alllane_lf(path_ego,X0_g,path_now,vehicle_left,vehicle_centre,vehicle_right)
        group_dict,ego_group = utils.formulate_gap_group(path_now,last_X,all_info,vehicle_left,vehicle_centre,vehicle_right)
        desired_group = decision_maker.decision_making(group_dict,start_group_str)
        path_d, path_dindex, C_label, sample,x_list,y_list,X0 = Decision_info(X0,X0_g,path_center,sample_center,x_center,y_center,boundary,desired_group,path_ego,path_now)
        C_label_additive = utils.inquire_C_state(C_label,desired_group)
        # last_desired_group = desired_group
        if C_label_additive == "Probe":
            path_d = path_ego
            path_dindex = path_now
            C_label_virtual = "K"
            _, xc, yc, samplesc = get_path_info(path_dindex)
            X0 =  repropagate(path_d,samplesc,xc,yc,X0_g,X0) 
        else:
            C_label_virtual = C_label

        path_desired.append(path_d)
        if  path_changed != path_dindex:
            mpc_controller.get_path_curvature(path=path_d)
            oS, oey = path_to_path_proj(oS, oey,path_changed,path_dindex)
            last_X = [ovx, ovy, owz, oS, oey, oepsi]

        path_changed = path_dindex

        ################################# Part of Metrics caculation #######################
        # S_obs calculation
        ego_rect = create_rectangle(X0_g[0], X0_g[1], mpc_controller.vehicle_length,mpc_controller.vehicle_width,X0_g[2])
        s_obs_min = surroundings.S_obs_calc(ego_rect)
        S_obs_record.append(s_obs_min)
        
        vel.append(X0[0])
        
        lane_state_record.append(C_label_additive)
        C_label_record.append(C_label)
        path_record.append(path_now)
        
        if i == 100:
            path_m, x_m, y_m, samples_m = get_path_info(1)
            s_m,ey_m,epsi_m = find_frenet_coord(path_m, x_m, y_m, samples_m,X0_g)
            progress_20 = s_m
            metrics_save(S_obs_record,initial_params,progress_20,progress_40,TTC_record,vel,acc,steer_record,lane_state_record,path_record,C_label_record,initial_vds,"100",round_)
        
        if i == 200:
            path_m, x_m, y_m, samples_m = get_path_info(1)
            s_m,ey_m,epsi_m = find_frenet_coord(path_m, x_m, y_m, samples_m,X0_g)
            progress_20 = s_m
            metrics_save(S_obs_record,initial_params,progress_20,progress_40,TTC_record,vel,acc,steer_record,lane_state_record,path_record,C_label_record,initial_vds,"200",round_)

        if i == 400:
            path_m, x_m, y_m, samples_m = get_path_info(1)
            s_m,ey_m,epsi_m = find_frenet_coord(path_m, x_m, y_m, samples_m,X0_g)
            progress_40 = s_m 
            metrics_save(S_obs_record,initial_params,progress_20,progress_40,TTC_record,vel,acc,steer_record,lane_state_record,path_record,C_label_record,initial_vds,"400",round_)
            break
        
        ################################# MPC solve and total env propogate ############           
        oa, od, ovx, ovy, owz, oS, oey, oepsi = mpc_controller.iterative_linear_mpc_control(X0, oa, od, dt, None, None, C_label,  X0_g, path_d, last_X, path_now, ego_group, path_ego, desired_group,vehicle_left,vehicle_centre,vehicle_right,path_dindex,C_label_additive,C_label_virtual) 
                                                            
        last_X = [ovx, ovy, owz, oS, oey, oepsi]
        
        u0 = [oa[0], od[0]]
        
        X0, X0_g, a_lon_lat, ds = dynamics.propagate(X0, u0, dt, X0_g, path_d, sample,x_list,y_list, boundary)
        
        acc.append(oa[0])
        steer_record.append(od[0])

        surroundings.total_update()
        
if __name__ == '__main__':
    rounds = [str(i) for i in range(0,200)]

    with Pool(processes=10) as pool:
        results = pool.map(one_round, rounds)

# rounds = [str(i) for i in range(28,100)]
# for round_ in rounds:
#     one_round(round_)
    