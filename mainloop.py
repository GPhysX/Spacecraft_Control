from math import *
import numpy as numpy
from spacecraft import *
import matplotlib.pyplot as plt
from pid import *




def main():
  sc = spacecraft(euler())  
  att = acc()
  #print sc.n_orbit()


  for i in range(0,100):
    u = att.control(sc.euler, np.matrix([0,0,0]).T)
    sc.Heun(u)
  for i in range(100,200):
    u = att.control(sc.euler, np.matrix([1,0,0]).T)
    sc.Heun(u)
  for i in range(200,300):
    u = att.control(sc.euler, np.matrix([0,1,0]).T)
    sc.Heun(u)
  for i in range(300,400):
    u = att.control(sc.euler, np.matrix([0,-1,0.1]).T)
    sc.Heun(u)
  for i in range(400,450):
    u = att.control(sc.euler, np.matrix([0,0,0]).T)
    sc.Heun(u)
  for i in range(450,800):
    u = att.control(sc.euler, np.matrix([-1,-1,-1]).T)
    sc.Heun(u)      

  theta1 = []
  theta2 = []
  theta3 = []
  t = [0]
  for e in sc.history:
    theta1.append(e[3,0])
    theta2.append(e[4,0])
    theta3.append(e[5,0])
    t.append(t[-1]+0.01)
  theta1.append(theta1[-1])
  theta2.append(theta2[-1])
  theta3.append(theta3[-1])
  plt.axis([0, 8, -3.14, 3.14])
  plt.plot(t, theta1)
  plt.plot(t, theta3)
  plt.plot(t, theta2)
  plt.show()

    #print (sc.euler.rot)





if __name__ == '__main__':
  main()