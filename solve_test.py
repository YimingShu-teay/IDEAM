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
from GPR.model_gp import *
from DecisionMaking.decision_params import *
from DecisionMaking.give_desired_path import *
from DecisionMaking.util import *
from DecisionMaking.util_params import *
from DecisionMaking.decision import *
from Prediction.surrounding_prediction import *
from progress.bar import Bar

Constraint_params = constraint_params()
mpc_controller = LMPC(**Constraint_params)
util_params_ = util_params()
utils = LeaderFollower_Uitl(**util_params_)
mpc_controller.set_util(utils)

file_dir = r"C:\Users\sym02\Desktop\Research\Extension\codes\decision_change_rear\solve_file\1722607609"

with open(file_dir,"rb") as file:
    data = pickle.load(file)

print(data.keys())
solve_content = data[249]
X0, oa, od, dt, GPR_vy, GPR_w, C_label, X0_g, path_d, last_X, path_now, ego_group, path_ego, desired_group,vehicle_left,vehicle_centre,vehicle_right,path_dindex,C_label_additive,C_label_virtual = solve_content

oa, od, ovx, ovy, owz, oS, oey, oepsi = mpc_controller.iterative_linear_mpc_control(X0, oa, od, dt, GPR_vy, GPR_w, C_label,  X0_g, path_d, last_X, path_now, ego_group, path_ego, desired_group,vehicle_left,vehicle_centre,vehicle_right,path_dindex,C_label_additive,C_label_virtual)

print("path_now=",path_now)
print("path_dindex=",path_dindex)
# print("C_label_additive=",C_label_additive)
# print("C_label=",C_label)
print("X0=",X0)
print("X0_g=",X0_g)
