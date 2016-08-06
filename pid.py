import numpy as np
from math import *
import os

class attitude(object):  
  def __init__(self, kp_att = np.matrix([1,1,1]).T, 
                kd_att = np.matrix([0.4,0.4,0.4]).T, 
                kI_att = np.matrix([0.05,0.05,0.05]).T, freq = 100):
    self.kp = -3*kp_att
    self.kd = -3*kd_att
    self.kI = -10*kI_att
    self.freq = freq
    self.error = np.matrix([0.,0.,0.]).T
    self.I = 0
    self.rt = rate()

  def control(self, state, setpoint):     
    att = state.attitude()
    rot = state.rot()
    old = self.error
    self.error = setpoint - att
    self.I += (self.error + old)  / float(self.freq) / 2.0     #trapezoidal integration scheme, O(h2)
    d = (self.error - old) * float(self.freq)                #finite difference, O(h)
    r = np.multiply(self.kp, self.error) + np.multiply(self.kd, d) + np.multiply(self.kI, self.I)
    print (self.error, os.linesep)#, r, os.linesep)
    return self.rt.control(rot, r)

class rate(object):  
  def __init__(self, kp_rate = np.matrix([10, 10, 10]).T, 
                kd_rate = np.matrix([4,4, 4]).T, 
                kI_rate = np.matrix([3,3,3]).T, freq = 100):
    self.kp = kp_rate
    self.kd = kd_rate
    self.kI = 0.1*kI_rate
    self.freq = freq
    self.error = np.matrix([0.,0.,0.]).T
    self.I = np.matrix([0.,0,0]).T
    self.acc = acc()

  def control(self, rot, setpoint):    
    old = self.error
    self.error = setpoint - rot
    self.I += (self.error + old)  / float(self.freq) / 2.0     #trapezoidal integration scheme, O(h2)
    d = (self.error - old) / float(self.freq)                #finite difference, O(h)
    a = np.multiply(self.kp, self.error) + np.multiply(self.kd, d) + np.multiply(self.kI, self.I)
    #print setpoint, rot
    return self.acc.control(rot, a)

class acc(object):  
  def __init__(self, kp_acc = np.matrix([124.5, 124.5, 0.7]).T, 
                kd_acc = np.matrix([12.40,12.40, 0.07]).T, freq = 100):
    self.kp = kp_acc
    self.kd = kp_acc * 0.4
    self.freq = freq
    self.error = np.matrix([0.,0.,0.]).T
    self.d = np.matrix([0.,0.,0.]).T
    self.old = np.matrix([0.,0.,0.]).T
    self.setpoint = np.matrix([0.,0.,0.]).T
    self.u = np.matrix([0.,0.,0.]).T
    self.I = np.matrix([0.,0.,0.]).T

  def control(self, rot, setpoint):
    self.d = (rot - self.old) * self.freq
    diff = self.setpoint - self.d
    self.I += diff
    #print self.d, setpoint
    self.setpoint = setpoint
    self.old = rot
    self.u = np.multiply(self.kp, self.setpoint) + np.multiply(self.I, self.kd)
    return self.u

