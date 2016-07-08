from math import *
import numpy as numpy
from spacecraft import *
import matplotlib.pyplot as plt




def main():
	sc = spacecraft(euler())	
	#print sc.n_orbit()
	plt.axis([0, 60, -0.03, 0.03])
	plt.ion()
	for i in range(0,600):
		sc.Heun()
		plt.scatter(sc.t, sc.euler.state()[1,0])
		plt.scatter(sc.t, sc.euler.state()[0,0])
		plt.draw()
		#print (sc.euler.rot)





if __name__ == '__main__':
	main()