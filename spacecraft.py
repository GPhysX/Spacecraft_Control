#
import numpy as np
from math import *
delta_t = 0.001

class spacecraft(object):

	def __init__(self, euler, ixx = 124.531, iyy = 124.586, izz = 0.704, orbit = 700000):
		self.ixx = ixx
		self.iyy = iyy
		self.izz = izz
		self.orbit = orbit 					#for now a constant, since only a circular orbit is used
		self.J = np.matrix([[ixx,0,0],[0,iyy,0],[0,0,izz]])
		self.euler = euler
		history = [self.euler]

	def Kinematic_euler(self, u, rot = 0):
		if rot == 0:
			rot = self.euler.rot					
		fx1 = self.euler.N * rot
		fx2 = np.cross(-self.J.I*self.euler.OMEGA_fn(rot), self.J*rot, axis = 0)
		fx = np.array([[fx1], [fx2]])
				#fx = np.matrix([[self.euler.N * rot],[np.cross(-self.J.I*self.euler.OMEGA_fn(rot), self.J*rot, axis = 0)]])
		print fx
		Gx = np.matrix([[np.zeros(3), np.zeros(3)],[self.J.I, self.J.I]])
		x_d = fx + Gx * u
		print x_d
		return x_d

	def Heun(self, u = np.matrix([0.01,0.01,0.01])):
		x_d = self.Kinematic_euler(u)
		rot_ = self.euler.rot + x_d * delta_t
		x_d2 = self.Kinematic_euler(u, rot_)
		rot = self.euler.rot + delta_t/2*(x_d + x_d2)
		self.euler.update_rot(rot)
		self.history.append(self.euler)











class euler(object):
	def __init__(self, theta1 = 0, theta2 = 0, theta3 = 0, theta1_d = 0, theta2_d = 0, theta3_d = 0):
		self.theta1 = theta1
		self.theta1_d = theta1_d
		self.theta2 = theta2
		self.theta2_d = theta2_d
		self.theta3 = theta3
		self.theta3_d = theta3_d
		self.attitude = np.matrix([theta1, theta2, theta3]).T
		self.rot = np.matrix([theta1_d, theta2_d, theta3_d]).T
		self.state = np.array([self.attitude.T, self.rot.T]).T
		self.OMEGA = np.matrix([[0, -theta3_d, theta2_d],[theta3_d, 0, -theta1_d],[-theta2_d, theta1_d, 0]])
		self.N = np.matrix([[1, sin(theta1)*tan(theta2), cos(theta1)*tan(theta2)],[0, cos(theta1), -sin(theta1)],[0, sin(theta1)/cos(theta2), cos(theta1)/cos(theta2)]])

	def update_attitude(theta1, theta2, theta3):
		self.theta1 = theta1
		self.theta2 = theta2
		self.theta3 = theta3
		self.attitude = np.matrix([theta1, theta2, theta3]).T
		self.N = np.matrix([[1, sin(theta1)*tan(theta2), cos(theta1)*tan(theta2)],[0, cos(theta1), -sin(theta1)],[0, sin(theta1)/cos(theta2), cos(theta1)/cos(theta2)]])


	def update_rot(theta1_d, theta2_d, theta3):
		self.theta1_d = theta1_d
		self.theta2_d = theta2_d
		self.theta3_d = theta3_d
		self.rot = np.matrix([theta1_d, theta2_d, theta3_d]).T
		self.state = np.array([self.attitude.T, self.rot.T]).T
		self.OMEGA = np.matrix([[0, -theta3_d, theta2_d],[theta3_d, 0, -theta1_d],[-theta2_d, theta1_d, 0]])
		
	def update_var(rot, att = 0):
		if not att == 0:
			self.theta1 = att[0]
			self.theta2 = att[1]
			self.theta3 = att[2]

		self.theta1_d = rot[0]
		self.theta2_d = rot[1]
		self.theta3_d = rot[2]

	def OMEGA_fn(self, rot):
		return np.matrix([[0, -rot[2], rot[1]],[rot[2], 0, -rot[0]],[-rot[1], rot[0], 0]])


