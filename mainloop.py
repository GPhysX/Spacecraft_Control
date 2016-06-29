from math import *
import numpy as numpy
from spacecraft import *


def main():
	sc = spacecraft(euler())
	#print sc.n_orbit()
	for i in range(0,150):
		sc.Heun()
		print (sc.euler.rot)





if __name__ == '__main__':
	main()