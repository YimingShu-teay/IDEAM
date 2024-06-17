import numpy as np
from multiprocessing import Process
from multiprocessing import Lock, Queue
from GPR.model_gp import *


GPR_vy = ModelGP(2)
GPR_w = ModelGP(2)

def GPR_vy_train():
    GPR_vy.train()

def GPR_w_train():
    GPR_w.train()
    
def Predict(estimation, GPR_vy, GPR_w, T):
    residual_vy = np.zeros((6,T))
    residual_w = np.zeros((6,T))
    for i in range(T):
        y_vy, _ = GPR_vy.predict(estimation[1,i],estimation[2,i])
        y_w, _ = GPR_w.predict(estimation[1,i],estimation[2,i])
        residual_vy[:,i] = y_vy[0]
        residual_w[:,i] = y_w[0]
