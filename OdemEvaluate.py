import numpy as np
import matplotlib.pyplot as plt
import math
import time


class RobotSensor(object):

	def __init__(self, T, WheelRadius = 0.075, W_w = ( 0, 0, 0 )):
			# encoder reading angular velocity each wheels
		self.w0, self.w1, self.w2 = W_w
		# wheel radius
		self.WheelRadius = WheelRadius

		self.T = T
		# IMU reading angular velocity about z axis
		self.R_w = self.getAngularVelocity()

	def getLinearVelocity(self):
		V_J = np.zeros((3,1))

		V_J[0,0] = self.w0 * self.WheelRadius
		V_J[1,0] = self.w1 * self.WheelRadius
		V_J[2,0] = self.w2 * self.WheelRadius

		return V_J

	def getAngularVelocity(self):
		T_inv = np.linalg.inv(self.T)
		# calculate V_r = [v,vn,w]
		V_r = np.matmul( T_inv, self.getLinearVelocity())
		self.R_w =  V_r[2,0]


		return self.R_w


class OdomEvaluate(object):

	def __init__(self, WheelRadius = 0.075, W_w = ( 0, 0, 0 ), d = 0.15, 
					  initPosition = (0.,0.,0.), isVisualize = False):

		self.x = initPosition[0]
		self.y = initPosition[1]
		self.theta = initPosition[2]

		# distance from wheel to center of robot
		self.radius = d

		self.isVisualize = isVisualize

		# Create T matrix for forward and inverse kinematics
		T = [ -math.sin(math.radians(60)), math.cos(math.radians(60)), self.radius,
					0, -1, self.radius,
					math.sin(math.radians(60)),math.cos(math.radians(60)), self.radius]
		self.T = np.array(T).reshape(3,3)

		# Initial transformation matrix, This matrix depend on theta of robot

		self.R_c_r = lambda theta : np.array([math.cos(theta), math.sin(theta),0,
											  -math.sin(theta),math.cos(theta),0,
											  0,0,1  ]).reshape(3,3)

		sensor = RobotSensor( self.T, WheelRadius = 0.075, W_w = W_w )
		
		self.V_J = sensor.getLinearVelocity()
		self.Omega = sensor.getAngularVelocity()

		# initial timestamp variable
		self.timestamp = None

	def evaluateOdom(self):
		
		currentTime = time.time()
		delta_t = currentTime - self.timestamp
		self.timestamp = currentTime

		R_c_r = self.R_c_r(self.theta)
		R_c_r_inv = np.linalg.inv(R_c_r)
		T_inv = np.linalg.inv(self.T)
		# calculate forward kinematics
		V_c = np.matmul( R_c_r_inv, np.matmul(T_inv, self.V_J))
		# calculate Odeometry
		self.x += float(V_c[0,0] * delta_t)
		self.y += float(V_c[1,0] * delta_t)
		self.theta += float(V_c[2,0] * delta_t)
		
		return self.x,self.y,self.theta

	def visualize(self,x,y,theta):
		plt.cla()
		plt.axis([-50, 50, -50, 50])
		plt.scatter(x, y)
		plt.quiver([x, x], [y, y], [math.cos(theta), math.cos(theta+math.pi/2)], [math.sin(theta), math.sin(theta+math.pi/2)], angles='xy', scale_units='xy', scale=0.1,color = ['r','b'])
		plt.pause(0.05)
		
	def calculateInverseKinematics(self,vx,vy,w,theta):

		V_c = np.array([vx,vy,w]).reshape(3,1)
		R_c_r = self.R_c_r(theta)

		V_J = np.matmul(self.T,np.matmul(R_c_r,V_c))

		print V_J

	def run(self):
		# initial start time
		self.timestamp = time.time()
		
		while(True):
			odom = self.evaluateOdom()
			print 'x:%5f, y:%5f, theta:%5f'%(odom[0],odom[1],math.degrees(odom[2])%360)
			if self.isVisualize:
				self.visualize(odom[0],odom[1],odom[2])
		plt.show()

if __name__ == "__main__":
	# Robot specifications
	WheelRadius = 0.075
	radiusWheelToCenter = 0.15

	# velocity and initial position
	v0 = -10
	v1 = 0
	v2 = 10
	intialPosition = (0,0,math.radians(0)) # x,y,theta(rad)

	odom = OdomEvaluate(WheelRadius =WheelRadius, W_w = ( v0 , v1, v2 ), d = radiusWheelToCenter, 
					   initPosition = intialPosition, isVisualize = True)
	odom.run()
	# odom.calculateInverseKinematics(5,0,0,math.radians(45))