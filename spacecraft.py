#
import numpy as np
from math import *
delta_t = 0.1

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
		Gx = np.array([[np.zeros(3), np.zeros(3)],[self.J.I, self.J.I]])
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

	def kin_euler(self, omega):
		theta1 = self.euler.theta1
		theta2 = self.euler.theta2
		theta3 = self.euler.theta3
		self.rot = 1/cos(2*theta1) * np.matrix([[cos(theta2), sin(theta1)*sin(theta2), cos(theta1)*sin(theta2)],[0, cos(theta1)*cos(theta2), -sin(theta1)*cos(theta2)],[0, sin(theta1), cos(theta1)]]) * omega

	def kin_quat(self, omega):
		q_d = 0.5 * self.quat.Q_omega() * omega

	def kin_MRP(self, omega):
		sd = 0.25 * MRP.N()*omega

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
		return np.matrix([[0, -rot[2], rot[1]],
			[rot[2], 0, -rot[0]],
			[-rot[1], rot[0], 0]])


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

		self.rot = np.matrix([self.q1d, self.q2d, self.q3d, self.q4d]).T
		self.q = np.matrix([self.q1, self.q2, self.q3, self.q4]).T

	def Q_omega(self):
		return ([[q4, -q3, q2, q1],[q3, q4, -q1, q2],
			[-q2, q1, q4, q3],
			[-q1, -q2, -q3, q4]])


class MRP(object):
	def __init__(self, s1 = 0, s2 = 0, s3 = 0):
		self.s1 = s1
		self.s2 = s2
		self.s3 = s3
		self.s1d = 0
		self.s2d = 0
		self.s3d = 0

	def s(self):
		return np.matrix([self.s1, self.s2, self.s3]).T

	def sd(self):
		return np.matrix([self.s1d, self.s2d, self.s3d]).T

	def N(self):
		s1 = self.s1
		s2 = self.s2
		s3 = self.s3
		s = self.s()
		return np.matrix([[ 1 - s()**2 + 2 * s1**2, -2*s3+2*s1*s2, 2*s2 + 2*s1*s3],
			[2* s3 + 2*s1*s2, 1 - s**2 + 2*s2**2, -2*s1+2*s2*s3],
			[-2*s2+2*s1*s3, 2*s1+2*s2*s3, 1-s**2 + 2*s3**2]])
