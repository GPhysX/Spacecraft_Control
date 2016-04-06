#
import numpy as np
from math import *

class Spacecraft(object):

	def __init__(self, ixx = 124.531, iyy = 124.586, izz = 0.704, orbit = 700000, orientation=orientation(), euler = euler()):
		self.ixx = ixx
		self.iyy = iyy
		self.izz = izz
		self.orbit = orbit 					#for now a constant, since only a circular orbit is used
		self.J = np.array([[ixx,0,0],[0,iyy,0],[0,0,izz]])

		self.orientation = orientation
		self.ang_accel = ang_accel
		self.ang_velocity = ang_velocity
		self.euler = euler



	def Kinematic_euler(self, u):
		fx = np.matrix([self.euler.N * self.euler.rot],[np.cross(-self.J.getI()*self.euler.OMEGA, self.J*self.euler.rot])
		Gx = np.matrix([np.zeros(3,3), np.zeros(3,3)],[self.J.getI(), self.J.getI()])
		x_d = fx + Gx * u
		











class euler(object):
	def __init__(self, theta1 = 0, theta2 = 0, theta3 = 0, theta1_d = 0, theta2_d = 0, theta3_d = 0):
		self.theta1 = theta1
		self.theta1_d = theta1_d
		self.theta2 = theta2
		self.theta2_d = theta2_d
		self.theta3 = theta3
		self.theta3_d = theta3_d
		self.attitude = np.array([theta1, theta2, theta3])
		self.rot = np.array([theta1_d, theta2_d, theta3_d])
		self.state = np.array([self.attitude, self.rot])
		self.OMEGA = np.array([0, -theta3_d, theta2_d],[theta3_d, 0, -theta1_d],[-theta2_d, theta1_d, 0])
		self.N = np.array([1, sin(theta1)*tan(theta2), cos(theta1)*tan(theta2)],[0, cos(theta1), -sin(theta1)],[0, sin(theta1)/cos(theta2), cos(theta1)/cos(theta2)])

	def update_attitude(theta1 = self.theta1, theta2 = self.theta2, theta3 = self.theta3):
		self.theta1 = theta1
		self.theta2 = theta2
		self.theta3 = theta3
		self.attitude = np.array([theta1, theta2, theta3])
		self.N = np.array([1, sin(theta1)*tan(theta2), cos(theta1)*tan(theta2)],[0, cos(theta1), -sin(theta1)],[0, sin(theta1)/cos(theta2), cos(theta1)/cos(theta2)])


	def update_rot(theta1_d = self.theta1_d, theta2_d = self.theta2_d, theta3 = self.theta3_d):
		self.theta1_d = theta1_d
		self.theta2_d = theta2_d
		self.theta3_d = theta3_d
		self.rot = np.array([theta1_d, theta2_d, theta3_d])
		self.state = np.array([self.attitude, self.rot])
		self.OMEGA = np.array([0, -theta3_d, theta2_d],[theta3_d, 0, -theta1_d],[-theta2_d, theta1_d, 0])
		

		
		