import numpy as np
deltaT = 0.01

class Kalman(object):

	def __init__(self, n_state_variables, n_measurements):
		super(Kalman, self).__init__()
		self.n_state_variables = n_state_variables
		self.n_measurements = n_measurements

		self.x = np.zeros((n_state_variables, 1))
		self.sigma = np.identity(n_state_variables) 
		self.F = np.identity(n_state_variables)
		self.F[0, 3] = deltaT
		self.F[1, 4] = deltaT
		self.F[2, 5] = deltaT
		self.G = np.identity(n_state_variables)
		self.R = np.identity(n_measurements)
		self.I = np.identity(n_state_variables)

		self.first = True

	def predict(self):
		self.x = np.dot(self.F, self.x)
		self.sigma = np.dot(self.F, np.dot(self.sigma, self.F.T))

	def update(self, Y):

		# Y: new sensor values
		w = Y - np.dot(self.G, self.x)
		S = np.dot(self.G, np.dot(self.sigma, self.G.T)) + self.R
		H = np.dot(self.sigma, np.dot(self.G.T, np.linalg.inv(S)))
		self.x = self.x + np.dot(H, w)
		self.sigma = self.sigma - np.dot(H, np.dot(S, H.T))