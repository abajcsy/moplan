import numpy as np
import matplotlib.pyplot as plt
import pylab
import matplotlib.image as mpimg

class RigidBody():
	# Constant quantities 
	mass = 0.0 			# mass M 
	Ibody = [] 			# Ibody 
	Ibodyinv = [] 		# I−1 body (inverse of Ibody)
	
	# State variables 
	x = [] 					# x(t) 
	R = [] 					# R(t) 
	P = []					# P(t)
	L = [] 					# L(t) 
	
	# Derived quantities (auxiliary variables) 
	Iinv = [] 				# I−1(t) 
	v = []					# v(t) 
	omega = []			# ω(t) 
	
	# Computed quantities 
	force = []  			# F(t) 
	torque = [] 			# τ(t) 

	def __init__(mass,x,R,P,L):
		self.mass = mass
		self.x = x