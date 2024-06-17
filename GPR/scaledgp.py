import numpy as np
import GPy

#数据经过标准化，减去均值并除以标准差的方式将数据转化为均值为0、标准差为1的标准正态分布。
class ScaledGP:
	def __init__(self,xdim=1,ydim=1):
		self.xdim=xdim  
		self.ydim=ydim  
		self.ystd = np.ones(ydim)
		self.ymean = np.zeros(ydim)
		self.xstd = np.ones(xdim)
		self.xmean = np.zeros(xdim)
		self.m = GPy.models.GPRegression(np.zeros((1,xdim)),np.zeros((1,ydim)))

    #将现有的数据进行训练
	def optimize(self,x,y,update_scaling=True, num_inducing=50):
		assert(x.shape[1] == self.xdim and y.shape[1] == self.ydim)
		assert(x.shape[0] > 0 and y.shape[0] > 0)
		
		xmean = self.xmean
		xstd = self.xstd
		ymean = self.ymean
		ystd = self.ystd
		if update_scaling:
			xmean,xstd = self.update_xscale(x)
			ymean,ystd = self.update_yscale(y)

		x = self.scalex(x,xmean,xstd)
		y = self.scaley(y,ymean,ystd)
		updated_model = GPy.models.GPRegression(x,y)
		# updated_model = GPy.models.SparseGPRegression(x,y,num_inducing=50)
		updated_model.optimize('bfgs')
		self.m = updated_model

		self.xmean = xmean
		self.xstd = xstd
		self.ymean = ymean
		self.ystd = ystd

	def predict(self,x):
		x = self.scalex(x,self.xmean,self.xstd)
		mean,var = self.m.predict_noiseless(x)
		mean = self.unscaley(mean,self.ymean,self.ystd)
		var = var * self.ystd
		if mean.size == 1:
			mean = mean[0,0]
			var = var[0,0]
		return mean,var

	def update_xscale(self,x):
		xmean = np.mean(x,axis=0)
		xstd = np.std(x,axis=0)

		return xmean,xstd

	def update_yscale(self,y):
		ymean = np.mean(y,axis=0)
		ystd = np.std(y,axis=0)

		return ymean, ystd

    #numpy数组中有一个元素为0则返回 x-xmean, 否则返回 (x-xmean)/xstd, 防止发生除以0的状况
	def scalex(self,x,xmean,xstd):
		if (xstd == 0).any():   
			return (x-xmean)
		else:
			return (x - xmean) / xstd

	def scaley(self,y,ymean,ystd):
		if (ystd == 0).any():
			return (y-ymean)
		else:
			return (y - ymean) / ystd
		
	def unscalex(self,x,xmean,xstd):
		if (xstd == 0).any():
			return x + xmean
		else:
			return x * xstd + xmean

	def unscaley(self,y,ymean,ystd):
		if (ystd == 0).any():
			return y + ymean
		else:
			return y * ystd + ymean