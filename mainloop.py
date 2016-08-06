from math import *
import numpy as numpy
from spacecraft import *
import matplotlib.pyplot as plt
from pid import *




def main():
  sc = spacecraft(euler())  
  att = attitude()
  #print([wW]+?)
  
  for i in range(0,40):#000):
    u = att.control(sc.euler, np.matrix([0.,0.,0.]).T)
    sc.Heun(u)
  for i in range(1000,2000):
    u = att.control(sc.euler, np.matrix([1.,0.,0.]).T)
    sc.Heun(u)
  for i in range(2000,3000):
    u = att.control(sc.euler, np.matrix([0.,1.,0.]).T)
    sc.Heun(u)
  for i in range(3000,4000):
    u = att.control(sc.euler, np.matrix([0.,-1.,1.]).T)
    sc.Heun(u)
  for i in range(4000,4500):
    u = att.control(sc.euler, np.matrix([0.,0.,0.]).T)
    sc.Heun(u)
  for i in range(4500,6000):
    u = att.control(sc.euler, np.matrix([-1.,-1.,-1.]).T)
    sc.Heun(u)      

  theta1 = []
  theta2 = []
  theta3 = []
  t = [0]
  for e in sc.history:
    theta1.append(e[0,0])
    theta2.append(e[1,0])
    theta3.append(e[2,0])
    t.append(t[-1]+0.01)
  theta1.append(theta1[-1])
  theta2.append(theta2[-1])
  theta3.append(theta3[-1])
  plt.axis([0, 60, -pi, pi])
  plt.plot(t, theta1)
  plt.plot(t, theta3)
  plt.plot(t, theta2)
  plt.show()


    #print([wW]+?)





if __name__ == '__main__':
  main()