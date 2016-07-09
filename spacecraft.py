#
import numpy as np
from math import *
delta_t = 0.01
Td = np.matrix([0.01,0.01,0.01]).T
Tc = np.matrix([0,0,0]).T

class spacecraft(object):

	def __init__(self, euler, ixx = 124.531, iyy = 124.586, izz = 0.704, orbit = 700000):
		self.ixx = ixx
		self.iyy = iyy
		self.izz = izz
		self.orbit = orbit 					#for now a constant, since only a circular orbit is used
		self.J = np.matrix([[ixx,0,0],[0,iyy,0],[0,0,izz]])
		self.euler = euler
		self.history = [self.euler.state()]
		self.t = 0

	def Kinematic_euler(self, u, rot = np.matrix([0,0,0])):
		if rot.all() == 0:
			rot = self.euler.rot()
		fx1 = np.dot(self.euler.N(), rot)
		fx2 = np.dot( np.dot(-self.J.I,self.euler.OMEGA(rot)), np.dot(self.J,rot))
		fx = np.bmat([[fx1], [fx2]])
		Gx = np.bmat([[np.zeros((3,3)), np.zeros((3,3))],[self.J.I, self.J.I]])
		#print(u.shape)
		#print(fx.shape)
		#print(Gx.shape)
		u = np.bmat([[Td], [u]])
		#print(np.dot(Gx, u).shape)
		#print(fx.shape)
		x_d = fx + np.dot(Gx, u)
		#print x_d.shape
		return x_d

	def Heun(self, u = np.bmat([[Td],[Tc]])):
		x_d = self.Kinematic_euler(u)
		#print self.euler.rot().shape
		#print x_d.shape
		#print self.euler.state().shape
		x = self.euler.state() + x_d * delta_t
		#print x.shape
		x_d2 = self.Kinematic_euler(u, self.euler.rot_(x))
		x = self.euler.state() + delta_t/2*(x_d + x_d2)
		#print x.shape
		#print x.T
		self.t += delta_t
		self.euler.update_state(x)
		self.history.append(self.euler.state())

	def kin_quat(self, omega):
		q_d = 0.5 * self.quat.Q_omega() * omega

	def kin_MRP(self, omega):
		sd = 0.25 * MRP.N()*omega

	def n_orbit(self):
		mu = 3.986004418*10**14
		orbit = float(self.orbit)
		return sqrt(mu/orbit**3)


class euler(object):
	def __init__(self, theta1 = 0, theta2 = 0, theta3 = 0, theta1_d = 0, theta2_d = 0, theta3_d = 0):
		self.theta1 = theta1
		self.theta1_d = theta1_d
		self.theta2 = theta2
		self.theta2_d = theta2_d
		self.theta3 = theta3
		self.theta3_d = theta3_d
				
	def update_attitude(self, att):
		self.theta1 = att[0]
		self.theta2 = att[1]
		self.theta3 = att[2]		

	def update_rot(self, rot):
		self.theta1_d = rot[0]
		self.theta2_d = rot[1]
		self.theta3_d = rot[2]
				
	def OMEGA(self, rot = np.matrix([0,0,0])):
		if rot.all() == 0:
			rot = self.rot()
		return np.matrix([[0, -rot[2], rot[1]],
			[rot[2], 0, -rot[0]],
			[-rot[1], rot[0], 0]])

	def N(self):
		return np.matrix([[1, sin(self.theta1)*tan(self.theta2), cos(self.theta1)*tan(self.theta2)],
			[0, cos(self.theta1), -sin(self.theta1)],
			[0, sin(self.theta1)/cos(self.theta2), cos(self.theta1)/cos(self.theta2)]])

	def attitude(self):
		return np.matrix([self.theta1, self.theta2, self.theta3]).T

	def rot(self):
		return np.matrix([self.theta1_d, self.theta2_d, self.theta3_d]).T

	def attitude_(self, state):
		return np.matrix([state[0][0], state[1][0], state[2][0]]).T

	def rot_(self, state):
		return np.matrix([state[3,0], state[4,0], state[5,0]]).T
	
	def state(self):
		return np.bmat([[self.attitude()], [self.rot()]])

	def x(self):
		return self.state()

	def update_state(self, state):
		self.theta1 = state[0,0]
		self.theta2 = state [1,0]
		self.theta3 = state [2,0]
		self.theta1_d = state[3,0]
		self.theta2_d = state[4,0]
		self.theta3_d = state[5,0]




class quaternions(object):
	def __init__(self, q1 = 0, q2 = 0, q3 = 0, q4 = 0):
		self.q1 = q1
		self.q2 = q2
		self.q3 = q3
		self.q4 = q4
		self.q1d = 0
		self.q2d = 0
		self.q3d = 0
		self.q4d = 0

	def Q_omega(self):
		return ([[q4, -q3, q2, q1],[q3, q4, -q1, q2],
			[-q2, q1, q4, q3],
			[-q1, -q2, -q3, q4]])

	def state(self):
		return np.matrix([q1,q2,q3,q4]).T

	def rot(self):
		return np.matrix([q1d, q2d, q3d, q4d]).T


class MRP(object):
	def __init__(self, s1 = 0, s2 = 0, s3 = 0):
		self.s1 = s1
		self.s2 = s2
		self.s3 = s3
		self.s1d = 0
		self.s2d = 0
		self.s3d = 0

	def attitude(self):
		return np.matrix([self.s1, self.s2, self.s3]).T

	def rot(self):
		return np.matrix([self.s1d, self.s2d, self.s3d]).T

	def N(self):
		s1 = self.s1
		s2 = self.s2
		s3 = self.s3
		s = self.s()
		return np.matrix([[ 1 - s()**2 + 2 * s1**2, -2*s3+2*s1*s2, 2*s2 + 2*s1*s3],
			[2* s3 + 2*s1*s2, 1 - s**2 + 2*s2**2, -2*s1+2*s2*s3],
			[-2*s2+2*s1*s3, 2*s1+2*s2*s3, 1-s**2 + 2*s3**2]])
