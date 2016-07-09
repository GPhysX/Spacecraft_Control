import numpy as np
from math import *

class pid(object):
	pass

class attitude(object):  
  def __init__(self, kp_att = np.matrix([0.7,0.7,0.7]).T, 
                kd_att = np.matrix([5,5,5]).T, 
                kI_att = np.matrix([0.05,0.05,0.05]).T, freq = 100):
    self.kp = kp_att
    self.kd = kd_att
    self.kI = kI_att
    self.freq = freq
    self.error = np.matrix([0,0,0]).T
    self.I = 0
    self.rt = rate()

  def control(self, state, setpoint):     
    att = state.attitude()
    rot = state.rot()
    old = self.error
    self.error = setpoint - att
    self.I += (self.error + old)  / float(self.freq) / 2.0     #trapezoidal integration scheme, O(h2)
    d = (self.error - old) / float(self.freq)                #finite difference, O(h)
    r = np.multiply(self.kp, self.error) + np.multiply(self.kd, d) + np.multiply(self.kI, self.I)
    return self.rt.control(rot, r)

class rate(object):  
  def __init__(self, kp_rate = np.matrix([4, 4, 4]).T, 
                kd_rate = np.matrix([4,4, 4]).T, 
                kI_rate = np.matrix([3,3,3]).T, freq = 100):
    self.kp = kp_rate
    self.kd = kd_rate
    self.kI = kI_rate
    self.freq = freq
    self.error = np.matrix([0,0,0]).T
    self.I = 0
    self.acc = acc()

  def control(self, rot, setpoint):    
    old = self.error
    self.error = setpoint - rot
    self.I += (self.error + old)  / float(self.freq) / 2.0     #trapezoidal integration scheme, O(h2)
    d = (self.error - old) / float(self.freq)                #finite difference, O(h)
    a = np.multiply(self.kp, self.error) + np.multiply(self.kd, d) + np.multiply(self.kI, self.I)
    return self.acc.control(rot, a)

class acc(object):  
  def __init__(self, kp_acc = np.matrix([-124.5, -124.5, -0.7]).T, 
                kd_acc = np.matrix([1240,1240, 7]).T, freq = 100):
    self.kp = kp_acc
    self.kd = kd_acc
    self.freq = freq
    self.error = np.matrix([0,0,0]).T
    self.d = np.matrix([0,0,0]).T

  def control(self, rot, setpoint):
    old = self.d
    old2 = self.error
    self.d = (rot - old) / float(self.freq)                #finite difference, O(h)
    self.error = (self.d - setpoint)
    dd = (self.error - old2) / float(self.freq)
    u = np.multiply(self.kp, self.error) + np.multiply(self.kd, dd)
    return u

