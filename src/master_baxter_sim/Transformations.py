#!/usr/bin/env python

import numpy as np

from math import (cos,sin)

class Transformations:
        #def __init__(self,):
	
	def Rotx(self,theta):
		R=np.matrix([[1.0, 0.0, 0.0],[0.0, cos(theta),  -sin(theta)], [0.0, sin(theta), cos(theta)]])
		return R
	def Roty(self,theta):
		R=np.matrix([[cos(theta), 0.0, sin(theta)],[0.0, 1.0,  0.0], [-sin(theta), 0.0, cos(theta)]])
		return R
	def Rotz(self,theta):
		R=np.matrix([[cos(theta), -sin(theta), 0.0],[sin(theta), cos(theta),  0.0], [0.0, 0.0, 1.0]])
		return R

	def quat_axis(self,axis,theta):
		res = np.matrix([[0],[0],[0],[0]], dtype=float) #matrix has to be of float type
		res[0] = cos(theta/2)
		res[1:,0] = axis * sin(theta/2)
		return res

	def quat_X(self, theta):
		axis = np.matrix([[1],[0],[0]])
		return self.quat_axis(axis,theta)

	def quat_Y(self, theta):
		axis = np.matrix([[0],[1],[0]])
		return self.quat_axis(axis,theta)

	def quat_Z(self, theta):
		axis = np.matrix([[0],[0],[1]])
		return self.quat_axis(axis,theta)




	







