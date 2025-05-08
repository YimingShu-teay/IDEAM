import sys
sys.path.append(r"C:\Users\sym02\Desktop\Research\Extension\codes\controller_improve2") 
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D
import matplotlib.pyplot as plt
import matplotlib.offsetbox as offsetbox
import os
from PIL import Image
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
import math
import imageio
from Model.Dynamical_model import *
from Model.params import params
from scipy.ndimage import rotate
import pickle
import time
import random

dirpath = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))

Params = params()

LENGTH = 4.0  # [m]
WIDTH = 1.9  # [m]
BACKTOWHEEL = 0.9  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.7  # [m]
WB = 1.9  # [m]
GOAL_DIS_X = 0.2
GOAL_DIS_Y = 0.1

def get_nparray_from_matrix(x):
    return np.array(x).flatten()

def normalize_angle(angle):

    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def plot_car(x, y, yaw_rad, length=5.0, width=2.0, color='blue', border_width=4, alpha=0.6):
    # Create the main car rectangle
    car = patches.Rectangle((x - length / 2, y - width / 2), length, width, color=color, zorder=2)
    
    # Create the outer border rectangle
    border_rect = patches.Rectangle((x - length / 2, y - width / 2), length, width, linewidth=border_width, edgecolor=color, facecolor='none', alpha=alpha, zorder=3)
    
    yaw_deg = np.rad2deg(yaw_rad)
    
    # Create the transformation
    transform = Affine2D().rotate_deg_around(x, y, yaw_deg) + plt.gca().transData
    
    # Apply the transformation to both rectangles
    car.set_transform(transform)
    border_rect.set_transform(transform)
    
    # Add both rectangles to the plot
    plt.gca().add_patch(car)
    plt.gca().add_patch(border_rect)



car_image = plt.imread(r"C:\Users\sym02\Desktop\Research\Extension\codes\decision_change_rear\carfigs\black&white.png")
def plot_car(x, y, yaw_rad, length=5.0, width=2.0):
    # 将偏航角从弧度转换为度数
    yaw_deg = np.rad2deg(yaw_rad)
    # 旋转图片
    rotated_image = rotate(car_image, angle=yaw_deg, reshape=True)

    # 调整缩放以匹配目标长宽比
    zoom_x = 0.4
    zoom_y = 0.28

    # 使用OffsetImage和extent进行缩放
    imagebox = offsetbox.OffsetImage(rotated_image, zoom=min(zoom_x, zoom_y))
    
    # 添加图片到图表中
    ab = offsetbox.AnnotationBbox(imagebox, (x, y), frameon=False)
    plt.gca().add_artist(ab)
    
car_image_surround_1 = plt.imread(r"C:\Users\sym02\Desktop\Research\Extension\codes\decision_change_rear\carfigs\yellow.png")
car_image_surround_2 = plt.imread(r"C:\Users\sym02\Desktop\Research\Extension\codes\decision_change_rear\carfigs\red.png")
# car_image_surround = random.choice([car_image_surround_1, car_image_surround_2])
def plot_car_surround(x, y, yaw_rad, i,lane):
    # 根据 i 的奇偶性选择图片
    if lane=="left" or lane=="right":
        if i % 2 == 1:
            car_image_surround = car_image_surround_1
        else:
            car_image_surround = car_image_surround_2
    elif lane=="center":
        if i % 2 == 1:
            car_image_surround = car_image_surround_2
        else:
            car_image_surround = car_image_surround_1  
    

    # 将偏航角从弧度转换为度数
    yaw_deg = np.rad2deg(yaw_rad)
    
    # 旋转图片
    rotated_image = rotate(car_image_surround, angle=yaw_deg, reshape=True)

    # 调整缩放以匹配目标长宽比
    zoom_x = 0.4
    zoom_y = 0.28

    # 使用OffsetImage和extent进行缩放
    imagebox = offsetbox.OffsetImage(rotated_image, zoom=min(zoom_x, zoom_y))
    
    # 添加图片到图表中
    ab = offsetbox.AnnotationBbox(imagebox, (x, y), frameon=False)
    plt.gca().add_artist(ab)


# def plot_car_on_trajectory(trajectory, car_image_path):
#     fig, ax = plt.subplots()
#     for x, y in trajectory:
#         imagebox = offsetbox.OffsetImage(car_image, zoom=0.3)  # 调整zoom参数来缩放车辆图像的大小
#         ab = offsetbox.AnnotationBbox(imagebox, (x, y), frameon=False)
#         ax.add_artist(ab)
#     ax.set_xlim(-1, 6)
#     ax.set_ylim(-1, 1)
#     plt.show()
    
def obstacle_plot(obs_x,obs_y,max_radius):
    circle = plt.Circle((obs_x, obs_y), max_radius, color='blue')
    plt.gca().add_patch(circle)


def png_count(addr):
    path =  addr
    files = os.listdir(path)   
    num_png = -1     
    for file in files:
        if file.endswith(".png"):
            num_png = num_png + 1
    return num_png


def animation_generation(addr,now_time):
    pic_num = png_count(addr)
    with imageio.get_writer(uri=addr+'\\{}.gif'.format(now_time), mode='I', duration=20) as writer:
        for i in range(pic_num):
            writer.append_data(imageio.imread((addr + "\\{}.png").format(i)))

def animation_generation2(addr, now_time):
    pic_num = png_count(addr)
    # 指定 GIF 文件的保存路径
    gif_path = f"{addr}\\{now_time}.gif"
    # 创建一个 GIF 写入器，注意设置适当的 duration（单位应该是秒）
    with imageio.get_writer(gif_path, mode='I', duration=0.2) as writer:  # 注意：duration 参数单位是秒，设置为 0.2 秒
        for i in range(pic_num):
            # 读取每个 PNG 文件
            image_path = f"{addr}\\{i}.png"
            with Image.open(image_path) as img:
                # 将图像转换为 'P' 模式，使用自适应调色板
                img_converted = img.convert('P', palette=Image.ADAPTIVE, colors=256)
                # 保存转换后的图像到临时内存
                img_converted.save(image_path, format='PNG')
                # 重新加载优化后的图像并添加到 GIF
                writer.append_data(imageio.imread(image_path))

def Fig_delete(addr):
    path = addr
    for root, dirs, files in os.walk(path):
        for name in files:
            if name.endswith(".png"):             
                os.remove(os.path.join(root, name))
                print("Delete File: " + os.path.join(root, name))
                

def find_frenet_coord(path_d,x_list,y_list,sample,x0_g):
    xy_stack = np.transpose(np.array([x_list,y_list])) - x0_g[0:2]
    d = np.linalg.norm(xy_stack,ord=2, axis=1)
    min_index = np.argmin(d)
    s = sample[min_index]
    ey = d[min_index]
    
    theta_r = path_d.get_theta_r(s)
    sign = (x0_g[1]-y_list[min_index])*np.cos(theta_r) - (x0_g[0]-x_list[min_index])*np.sin(theta_r)
    if sign > 0:
        pass
    elif sign <0:
        ey = -ey
    
    epsi = normalize_angle(x0_g[2] - theta_r)
    
    return s,ey,epsi


def clac_last_X(oa,od,T,path_now,dt,NX,x0,x0_g):
    xbar = np.zeros((NX, T + 1))
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]
    oa = [0.0] * T
    od = [0.0] * T
    dynamics = Dynamic(**Params)
    k_profile = np.zeros((1, T + 1))
    for (ai, di, i) in zip(oa, od, range(1, T + 1)):
        u0 = [ai,di]
        x0, x0_g = dynamics.propagate_iter(x0, u0, path_now.get_k(x0[3]), dt, x0_g)
        xbar[0, i] = x0[0]
        xbar[1, i] = x0[1]
        xbar[2, i] = x0[2]
        xbar[3, i] = x0[3]
        xbar[4, i] = x0[4]
        xbar[5, i] = x0[5]
        k_profile[0, i] = path_now.get_k(xbar[3, i]) 

    return xbar[0,:], xbar[1,:], xbar[2,:], xbar[3,:], xbar[4,:], xbar[5,:]

def get_future_trajectory(oS,oey,path_d,T):
    x_g, y_g, yaw_g = np.zeros(T), np.zeros(T), np.zeros(T)
    
    for i in range(T):
        xi,yi = path_d.get_cartesian_coords(oS[i], oey[i])
        yawi = path_d.get_theta_r(oS[i])
        x_g[i] = xi
        y_g[i] = yi
        yaw_g[i] = yawi 
        
    return x_g, y_g, yaw_g

#来自AMR_MPC-CBF
def transformProj2Orig(s, l, theta_tilde, path):
    '''
    输入：
    s,l,theta_tilde: frenet坐标系下的坐标 
    path:当前的道路
    输出：
    笛卡尔坐标系下的坐标
    '''
    X = [0] * len(s)
    Y = [0] * len(s)
    THETA = [0] * len(s)
    for i in range(len(s)):
        res = path(s[i])
        (x1, y1) = res
        theta_r = path.get_theta_r(s[i])
        
        x = x1 - np.sin(theta_r)*l[i]
        y = y1 + np.cos(theta_r)*l[i]
        
        theta = theta_tilde[i] + theta_r
        X[i] = x
        Y[i] = y
        THETA[i] = theta
    return (X, Y, THETA)

#来自AMR_MPC-CBF
def drawCar(x, y, theta, X_horizon, vehicle_length, vehicle_width, path, color):
    
    h = vehicle_length
    h2 = vehicle_width
    half_edge = vehicle_width/2
    t1 = plt.Polygon([[x+ (1/2)*h*np.cos(theta)- half_edge*np.sin(theta), y+(1/2)*h*np.sin(theta)+ half_edge*np.cos(theta)],  [x + half_edge*np.sin(theta)+(1/2)*h*np.cos(theta), y- half_edge*np.cos(theta)+(1/2)*h*np.sin(theta)], [x + half_edge*np.sin(theta)-(1/2)*h*np.cos(theta), y - half_edge*np.cos(theta)-(1/2)*h*np.sin(theta)]], color=color)
    plt.gca().add_patch(t1)
    t1 = plt.Polygon([[x + half_edge*np.sin(theta)-(1/2)*h*np.cos(theta), y - half_edge*np.cos(theta)-(1/2)*h*np.sin(theta)], [x+ (1/2)*h*np.cos(theta)- half_edge*np.sin(theta), y+(1/2)*h*np.sin(theta)+ half_edge*np.cos(theta)], [x - half_edge*np.sin(theta)-(1/2)*h*np.cos(theta), y+ half_edge*np.cos(theta)-(1/2)*h*np.sin(theta)]], color=color)
    plt.gca().add_patch(t1)
    t1 = t1 = plt.Polygon([[x+ (1/2)*h*np.cos(theta), y+ (1/2)*h*np.sin(theta)], [x - half_edge*np.sin(theta)-(1/2)*h*np.cos(theta), y+ half_edge*np.cos(theta)-(1/2)*h*np.sin(theta)], [x + half_edge*np.sin(theta)-(1/2)*h*np.cos(theta), y - half_edge*np.cos(theta)-(1/2)*h*np.sin(theta)]], color='green')
    plt.gca().add_patch(t1)

    (_x, _y, _theta) = transformProj2Orig(X_horizon[:,0], X_horizon[:,1], X_horizon[:,2], path)
    plt.plot(_x, _y, '-r', linewidth=0.5)

#来自AMR_MPC-CBF
def drawObstacles(obs, path, vehicle_length, vehicle_width,x_range,y_range,a,b):
    obs = transformProj2Orig([obs[0]], [obs[1]], [obs[2]], path)
    h = vehicle_length
    h2 = vehicle_width
    half_edge = vehicle_width/2

    x = obs[0][0]
    y = obs[1][0]
    theta = obs[2][0]

    t2 = plt.Polygon([[x+ (1/2)*h*np.cos(theta)- half_edge*np.sin(theta), y+(1/2)*h*np.sin(theta)+ half_edge*np.cos(theta)],  [x + half_edge*np.sin(theta)+(1/2)*h*np.cos(theta), y- half_edge*np.cos(theta)+(1/2)*h*np.sin(theta)], [x + half_edge*np.sin(theta)-(1/2)*h*np.cos(theta), y - half_edge*np.cos(theta)-(1/2)*h*np.sin(theta)]], color='green',zorder=2)
    plt.gca().add_patch(t2)
    t2 = plt.Polygon([[x + half_edge*np.sin(theta)-(1/2)*h*np.cos(theta), y - half_edge*np.cos(theta)-(1/2)*h*np.sin(theta)], [x+ (1/2)*h*np.cos(theta)- half_edge*np.sin(theta), y+(1/2)*h*np.sin(theta)+ half_edge*np.cos(theta)], [x - half_edge*np.sin(theta)-(1/2)*h*np.cos(theta), y+ half_edge*np.cos(theta)-(1/2)*h*np.sin(theta)]], color='green',zorder=2)
    plt.gca().add_patch(t2)
    
    delta = 0.025
    xrange = np.arange(x_range[0], x_range[1], delta)
    yrange = np.arange(y_range[0], y_range[1], delta)
    X, Y = np.meshgrid(xrange,yrange)
    F = ((np.cos(-theta)*(X-x)-np.sin(-theta)*(Y-y))**2 / (a*h)**2 ) + ((np.sin(-theta)*(X-x)+np.cos(-theta)*(Y-y))**2 / (b*h2)**2)
    plt.contour(X, Y, (F), [1], linestyles='dashed', linewidths=0.5, colors='blue',zorder=1)

def states_print(x0,x0_g,oa,od):
    print("vx=",x0[0])
    print("vy=",x0[1])
    print("w=",x0[2])
    print("s=",x0[3])
    print("ey=",x0[4])
    print("epsi=",x0[5])
    print("Psi=",x0_g[2])
    print("oa=",oa[0])
    print("od=",od[0])

    
def plot_error(t_list,ey_list,epsi_list,vx_list,vy_list,x_label):
    plt.subplot(2, 2, 1)
    plt.title("ey")
    plt.plot(t_list,ey_list)
    plt.axvline(x_label)
    plt.subplot(2, 2, 2)
    plt.title("epsi")
    plt.plot(t_list,epsi_list)
    plt.axvline(x_label)
    plt.subplot(2, 2, 3)
    plt.title("vx")
    plt.plot(t_list,vx_list)
    plt.axvline(x_label)
    plt.subplot(2, 2, 4)
    plt.title("vy")
    plt.plot(t_list,vy_list)
    plt.axvline(x_label)
    plt.xlabel("t")
    plt.ylabel("error")
    plt.savefig((dirpath+"\\figsave\{}".format("error")),dpi=600)




#抛物线形式
# def get_lamda_k(k,Th):
#     lambda_k = -k**2/Th**2+1
#     return lambda_k

#sigmoid函数形式
def get_lamda_k(k,Th):
    lambda_k = 1 - 1/(1 + np.exp((k-10)/5))
    return lambda_k

def solve_return_deal(oa,od):
    oa_ = oa[0]
    od_ = od[0]
    return oa_, od_

def model_object_return():
    model = Dynamic(**Params)
    return model

def get_disturbance(psi,vx,vy):
    v = np.sqrt(vx**2 + vy**2)
    ax = (0.5 * 0.4 * 3 * 1.225 * v**2) / 1292
    ay = (0.5 * 0.2 * 2.5 * 1.225 * v**2) / 1292
    return ax, ay


def create_rectangle(center_x, center_y, width, height, psi):
    cos_angle = math.cos(psi)
    sin_angle = math.sin(psi)
    
    corners = [
        (-width / 2, height / 2),
        (width / 2, height / 2),
        (width / 2, -height / 2),
        (-width / 2, -height / 2)
    ]
    
    global_corners = []
    for cx, cy in corners:
        global_x = center_x + cos_angle * cx - sin_angle * cy
        global_y = center_y + sin_angle * cx + cos_angle * cy
        global_corners.append((global_x, global_y))
    
    return Polygon(global_corners)


def metrics_save(S_obs_record,initial_params,progress_20,progress_40,TTC_record,vel,vys,ys,acc,steer_record,lane_state_record,path_record,C_label_record,initial_vds,iter_num,round_=None,comparison=None):
    metrics_dict = {"S_obs_record":S_obs_record,"initial_params":initial_params,"progress_20":progress_20,"progress_40":progress_40,"TTC_record":TTC_record,"vel":vel,"acc":acc,"steer_record":steer_record,"lane_state_record":lane_state_record,"path_record":path_record,"C_label_record":C_label_record,"initial_vds":initial_vds,"vys":vys,"ys":ys}
    if round_ is None:
        time_now = str(time.time()) + iter_num
    else:
        time_now = round_ + "_" + iter_num
    if comparison is not None:
        file = open("C:\\Users\\sym02\\Desktop\\Research\\Extension\\codes\\decision_change_rear\\"+ comparison + "\\file_save\\" + time_now, 'wb')
    else:
        file = open("C:\\Users\\sym02\\Desktop\\Research\\Extension\\codes\\decision_change_rear\\file_save\\" + time_now, 'wb')
    pickle.dump(metrics_dict, file)
    file.close()
    
def get_reference(path,oS,oey):
    refer_path = np.zeros((len(oS),4))
    for i in range(len(oS)):
        coord_i = path.get_cartesian_coords(oS[i],oey[i])
        refer_path[i,0] = coord_i[0]
        refer_path[i,1] = coord_i[1]
    return refer_path
    
def curve_calc(path,oS,oey):
    refer_path = get_reference(path,oS,oey)
    for i in range(len(refer_path)):
        if i == 0:
            dx = refer_path[i+1,0] - refer_path[i,0]
            dy = refer_path[i+1,1] - refer_path[i,1]
            ddx = refer_path[2,0] + refer_path[0,0] - 2*refer_path[1,0]
            ddy = refer_path[2,1] + refer_path[0,1] - 2*refer_path[1,1]
        elif i == (len(refer_path)-1):
            dx = refer_path[i,0] - refer_path[i-1,0]
            dy = refer_path[i,1] - refer_path[i-1,1]
            ddx = refer_path[i,0] + refer_path[i-2,0] - 2*refer_path[i-1,0]
            ddy = refer_path[i,1] + refer_path[i-2,1] - 2*refer_path[i-1,1]

        else:      
            dx = refer_path[i+1,0] - refer_path[i,0]
            dy = refer_path[i+1,1] - refer_path[i,1]
            ddx = refer_path[i+1,0] + refer_path[i-1,0] - 2*refer_path[i,0]
            ddy = refer_path[i+1,1] + refer_path[i-1,1] - 2*refer_path[i,1]

        refer_path[i,2]=math.atan2(dy,dx) # yaw
        refer_path[i,3]=(ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2)) 
    curve = refer_path[:,3]
    return curve


def save_sovle_content(file_dir,content,index):
    if os.path.exists(file_dir):
        with open(file_dir,"rb") as file:
            data = pickle.load(file)
    else:
        data = {}
    
    data[index] = content
    
    with open(file_dir,"wb") as file:
        pickle.dump(data,file)
