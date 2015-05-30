#!/usr/bin/python
import roslib
import rospy
from matplotlib.pyplot import *
from geometry_msgs.msg import WrenchStamped
import numpy as np
import math as m
import tf

#first_time = True
save_array = True
wrench = WrenchStamped()

# --- Kalman Filter initialization ---
wStd = 0.01
vStd = 500
A = np.eye(3)
A = np.mat(A)
I = np.eye(3)
I = np.mat(I)
r_hat_k = np.matrix([[0], [0], [0]])
P_k = np.eye(3)
P_k = np.mat(P_k)
Q = np.eye(3)*(m.pow(wStd,2))
Q = np.mat(Q)
R = np.eye(3)*(m.pow(vStd,2))
R = np.mat(R)
H = np.zeros((3,3))
H = np.mat(H)
z_k = np.zeros((3,1))

# Array vectors
r_hat_A = np.empty([3,0])
Fx = np.array([])
Fy = np.array([])
Fz = np.array([])
Tx = np.array([])
Ty = np.array([])
Tz = np.array([])


def wrench_callback(msg):
	global wrench, Torque, H
	global r_hat_A, Fx, Fy, Fz, Tx, Ty, Tz
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

	r_hat_k = Kalman_filter(H, z_k)
	if save_array == True:
		r_hat_A = np.hstack((r_hat_A, r_hat_k))
		Fx = np.append(Fx, wrench.wrench.force.x)
        Fy = np.append(Fy, wrench.wrench.force.y)
        Fz = np.append(Fz, wrench.wrench.force.z)
        Tx = np.append(Tx, wrench.wrench.torque.x)
        Ty = np.append(Ty, wrench.wrench.torque.y)
        Tz = np.append(Tz, wrench.wrench.torque.z) 
	return

def Kalman_filter(H, z_k):
	global A, r_hat_k, P_k, Q, R, I
	z_k = np.mat(z_k)
	# -------- Prediction Step --------
	r_hat_k = A * r_hat_k # [3x1]
	#print ('Pred r_hat_k', r_hat_k)
	# print ('type:', type(r_hat_k))
	P_k = P_k + Q # [3x3]; A * P_k * A' + Q; A = A' in this case
	#print ('Pred P_K', P_k)
	# print ('type(P_K)', type(P_k))
	#print ('H', H)
	# print ('type(H)', type(H))
	# -------- Update Step -----------
	# print('Breaking down Kalman gain')
	# test1 = (H * P_k * (-H)) # Correct result
	# test2 = (H * P_k * (-H)) + R #Correct result
	# test3 = np.linalg.inv( (H * P_k * (-H)) + R )
	# test4 = P_k * (-H) # Correct
	# test5 = P_k * (-H) * np.linalg.inv( (H * P_k * (-H)) + R )
	# print('test1: ', test1)
	# print('test2: ', test2)
	# print('test3: ', test3)
	# print('test4: ', test4)
	# print('test5: ', test5)
	# print('type test1', type(test1))
	# print('type test2', type(test2))
	# print('type test3', type(test3))
	# print('type test4', type(test4))
	# print('type test5', type(test5))
	K_k = P_k * (-H) * np.linalg.inv( (H * P_k * (-H)) + R ) # [3x3]; PH'(HPH'+R)-1; H'=-H
	r_hat_k = r_hat_k + K_k * ( z_k - H * r_hat_k ) # [3x1]
	#print("r_hat_k", r_hat_k)
	P_k = (I - K_k * H) * P_k # [3x3]
	
	return r_hat_k

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
	raw_input("\n Press key to continue")
	if save_array == True:
		r_x = np.array(r_hat_A[0,:])
		r_y = np.array(r_hat_A[1,:])
		r_z = np.array(r_hat_A[2,:])

		print('size(r_x)', np.size(r_x, 1))
		X_axis = np.linspace(0,(np.size(r_x, 1)-1),np.size(r_x, 1))
		X_for_r = np.array([X_axis])
		print('len(X_axis)',len(X_axis))
		print("len(Fx)",len(Fx))


		figure(1)
		subplot(311)
		plot(X_axis, Fx, 'r'), title('Fx'), ylabel('[N]')
		subplot(312)
		plot(X_axis, Fy, 'g'), title('Fy'), ylabel('[N]')
		subplot(313)
		plot(X_axis, Fz, 'b'), title('Fz'), ylabel('[N]')

		figure(2)
		subplot(311)
		plot(X_axis, Tx, 'r'), title('Tx'), ylabel('[Nm]')
		subplot(312)
		plot(X_axis, Ty, 'g'), title('Ty'), ylabel('[Nm]')
		subplot(313)
		plot(X_axis, Tz, 'b'), title('Tz'), ylabel('[Nm]')


		figure(3)
		subplot(311)
		plot(X_axis , np.squeeze(r_x), 'r'), title('Estimate of R_x'), ylabel('[m]')
		subplot(312)
		plot(X_axis, np.squeeze(r_y), 'g'), title('Estimate of R_y'), ylabel('[m]')
		subplot(313)
		plot(X_axis, np.squeeze(r_z), 'b'), title('Estimate of R_z'), ylabel('[m]')

		show()
