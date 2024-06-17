import numpy as np
from GPR.scaledgp import *


class ModelGP:
    def __init__(self,xdim,ydim=6,verbose=True):

        self.m = ScaledGP(xdim=xdim,ydim=ydim)
        self.y = np.zeros((0,ydim))  #数据集中的label集合
        self.z = np.zeros((0,xdim))  #数据集中的feature vector集合
        self.N_data = 1000

        self.model_trained = False
        self.verbose = verbose

    #GPR model的输入 z = [vx, vy, w, ax, delta]
    def make_input(self,vy_ref,w_ref):
        z = np.zeros((1,2))
        z[0,0] = vy_ref
        z[0,1] = w_ref    
        return z
    
    #给定z，进行模型预测
    def predict(self,vy_ref,w_ref):
        z = self.make_input(vy_ref,w_ref)
        y, var = self.m.predict(z)
        return y, var.T

    #数据集进行训练
    def train(self):
        if self.z.shape[0] > 0:
            self.m.optimize(self.z,self.y)
            self.model_trained = True

    def add_data(self,x_observed,x_clac,delta,feature_label):
        select_matrix = np.zeros((6,6))
        select_matrix[feature_label,feature_label] = 1
        ynew = select_matrix.dot(x_observed - x_clac)
        ynew = ynew[np.newaxis,:]
        
        znew = self.make_input(x_observed[1],x_observed[2])
        
        self.y = np.concatenate((self.y,ynew))
        self.z = np.concatenate((self.z,znew))

        # throw away old samples if too many samples collected.
        if self.y.shape[0] > self.N_data:
            self.y = self.y[-self.N_data:,:]
            self.z = self.z[-self.N_data:,:]
            # self.y = np.delete(self.y,random.randint(0,self.N_data-1),axis=0)
            # self.z = np.delete(self.z,random.randint(0,self.N_data-1),axis=0)

        if self.verbose:
            print("ynew",ynew)
            print("znew",znew)
            print("n data:", self.y.shape[0])
            # print("prediction error:", self.predict_error)
            # print("predict var:", self.predict_var)
