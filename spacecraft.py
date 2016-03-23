#

class Spacecraft(object):


	def __init__(self, ixx = 124.531, iyy = 124.586, izz = 0.704, orbit = 700000, orientation=orientation(), ang_velocity = ang_velocity(), ang_accel = ang_accel() ):
		self.ixx = ixx
		self.iyy = iyy
		self.izz = izz
		self.orbit = orbit 					#for now a constant, since only a circular orbit is used
		self.J = np.array([[ixx,0,0],[0,iyy,0],[0,0,izz]])
		self.orientation = orientation
		self.ang_accel = ang_accel
		self.ang_velocity = ang_velocity
