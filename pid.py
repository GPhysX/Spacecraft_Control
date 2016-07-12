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
                kd_rate = np.matrix([0.4,0.4, 0.4]).T, 
                kI_rate = np.matrix([0.,0.,0.]).T, freq = 100):
    self.kp = kp_rate
    self.kd = kd_rate
    self.kI = kI_rate
    self.freq = freq
    self.error = np.matrix([0,0,0]).T
    self.I = 0
    self.acc = acc()

  def control(self, state, setpoint): 
    rot = state.rot()   
    old = self.error
    self.error = setpoint - rot
    self.I += (self.error + old)  / float(self.freq) / 2.0     #trapezoidal integration scheme, O(h2)
    d = -(self.error - old) * float(self.freq)                #finite difference, O(h)
    a = np.multiply(self.kp, self.error) + np.multiply(self.kd, d) + np.multiply(self.kI, self.I)
    return self.acc.control(state, a)

class acc(object):  
  def __init__(self, kp_acc = 1, 
                kd_acc = -0, kI_acc = 100, freq = 100, J = np.matrix([124.5, 124.5, 0.7]).T):
    self.kp = kp_acc * J
    self.kd = kd_acc * J
    self.kI = kI_acc * J
    self.freq = freq
    self.error = np.matrix([0,0,0]).T
    self.d = np.matrix([0,0,0]).T
    self.old = np.matrix([0,0,0]).T
    self.I = np.matrix([0,0,0]).T

  def control(self, state, setpoint):
    rot = state.rot()
    old2 = self.error
    self.d = (rot - self.old) * float(self.freq)                #finite difference, O(h)
    print self.d, setpoint, self.I
    self.error = (setpoint - self.d)
    self.I += (self.error + old2)  / float(self.freq) / 2.0
    self.old = rot
    dd = (self.error - old2) * float(self.freq)
    u = np.multiply(self.kp, self.error) + np.multiply(self.kd, dd) + np.multiply(self.kI, self.I)
    #print u
    return u

