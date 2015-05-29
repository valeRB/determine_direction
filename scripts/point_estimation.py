#!/usr/bin/python
import roslib
import rospy
from matplotlib.pyplot import *
from geometry_msgs.msg import WrenchStamped
import numpy as np
import math as m
import tf

first_time = True
wrench = WrenchStamped()

# --- Kalman Filter initialization ---
wStd = 0.1
vStd = 100
A = np.eye(3)
A = np.mat(A)
I = np.eye(3)
I = np.mat(I)
r_hat = np.matrix([[0], [0], [0.5]])
P_k = np.eye(3)
P_k = np.mat(P_k)
Q = np.eye(3)*(m.pow(wStd,2))
Q = np.mat(Q)
R = np.eye(3)*(m.pow(vStd,2))
R = np.mat(R)
H = np.zeros((3,3))
H = np.mat(H)
z_k = np.zeros((3,1))

def wrench_callback(msg):
	global wrench, Torque, H
	wrench = msg
	H[0,1] = wrench.wrench.force.z
	H[0,2] = -wrench.wrench.force.y
	H[1,0] = -wrench.wrench.force.z
	H[1,2] = wrench.wrench.force.x
	H[2,0] = wrench.wrench.force.y
	H[2,1] = -wrench.wrench.force.x
	z_k[0] = wrench.wrench.torque.x
	z_k[1] = wrench.wrench.torque.y
	z_k[2] = wrench.wrench.torque.z
	
	Kalman_filter(H, z_k)
	#r_hat_k = Kalman_filter(H, z_k)
	return

def Kalman_filter(H, z_k):
	global A, r_hat, P_k, Q, R, I
	# Prediction Step
	print('---- Prediction ----')
	r_hat_k = A * r_hat # [3x1]
	print ('Pred r_hat_k', r_hat_k)
	print ('type:', type(r_hat_k))
	P_k = P_k + Q # [3x3]; A * P_k * A' + Q; A = A' in this case
	print ('Pred P_K', P_k)
	print ('H', H)
	print ('type(P_K)', type(P_k))
	# Update Step
	print('---- Update ----')
	print('Breaking down Kalman gain')
	test1 = (H * P_k * (-H))
	test2 = (H * P_k * (-H)) + R
	test3 = np.linalg.inv( (H * P_k * (-H)) + R )
	test4 = P_k * (-H) 
	test5 = P_k * (-H) * np.linalg.inv( (H * P_k * (-H)) + R )
	print('test1: ', test1)
	print('test2: ', test2)
	print('test3: ', test3)
	print('test4: ', test4)
	print('test5: ', test5)
	print('type test1', type(test1))
	print('type test2', type(test2))
	print('type test3', type(test3))
	print('type test4', type(test4))
	print('type test5', type(test5))
	#K_k = P_k * (-H) * np.linalg.inv( (H * P_k * (-H)) + R ) # [3x3]; PH'(HPH'+R)-1; H'=-H
	#r_hat_k = r_hat_k + K_k * ( z_k - H * r_hat_k ) # [3x1]
	#P_k = (I - K_k * H) * P_k # [3x3]



	
	


	return

def point_estimation():
	rospy.init_node('point_estimation', anonymous=True)
	rospy.Subscriber("/ft_transformed/lef_arm", WrenchStamped, wrench_callback)
	rospy.spin()
	return

if __name__ == '__main__':

	point_estimation()

	try:
		rate = rospy.Rate(10.0)
		while not rospy.is_shutdown():
			rate.sleep()

	except KeyboardInterrupt:
		pass