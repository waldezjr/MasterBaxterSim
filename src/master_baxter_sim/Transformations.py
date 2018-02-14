#!/usr/bin/env python

import numpy

from math import (cos,sin)

class Transformations:
        #def __init__(self,):
	
	def Rotx(self,theta):
		R=numpy.matrix([[1.0, 0.0, 0.0],[0.0, cos(theta),  -sin(theta)], [0.0, sin(theta), cos(theta)]])
		return R
	def Roty(self,theta):
		R=numpy.matrix([[cos(theta), 0.0, sin(theta)],[0.0, 1.0,  0.0], [-sin(theta), 0.0, cos(theta)]])
		return R
	def Rotz(self,theta):
		R=numpy.matrix([[cos(theta), -sin(theta), 0.0],[sin(theta), cos(theta),  0.0], [0.0, 0.0, 1.0]])
		return R






	







