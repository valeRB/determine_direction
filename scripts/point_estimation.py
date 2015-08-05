#!/usr/bin/python
import roslib
import rospy
from matplotlib.pyplot import *
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
import numpy as np
import math as m
import tf


save_array = True
print_once = True
show_error = False
msg_being_published = False
wrench = WrenchStamped()
R_vect = PointStamped()
true_point = PointStamped()
pub_R = rospy.Publisher("/KF/point_estimation", PointStamped)
pub_theta = rospy.Publisher("/KF/theta", Float32)
pub_t = rospy.Publisher("/KF/true_point", PointStamped)

# --- True point ---
true_point.point.x = 0
true_point.point.y = 0
true_point.point.z = 0.64

# --- Kalman Filter Initialization x = [rx, ry, rz]
r_hat_k = np.matrix([[0], [0], [0.5]])
wStd = 0.01
vStd = 50
start_pt = 0#13000 
X_value =0#-0.365  # [m]
Z_value = 0#0.62 # [m]

response_time= 10700

wStd_y = wStd #0.001 
A = np.eye(3)
A = np.mat(A)
I = np.eye(3)
I = np.mat(I)

P_k = np.eye(3)
P_k = np.mat(P_k)
Q = np.eye(3)*(m.pow(wStd,2))
Q = np.mat(Q)
Q[1,1] = (m.pow(wStd_y,2))
R = np.eye(3)*(m.pow(vStd,2))
R = np.mat(R)
H = np.zeros((3,3))
H = np.mat(H)
z_k = np.zeros((3,1))

# Array vectors
r_hat_A = np.empty([3,0])
theta_A = np.array([])
Fx = np.array([])
Fy = np.array([])
Fz = np.array([])
Tx = np.array([])
Ty = np.array([])
Tz = np.array([])


def wrench_callback(msg):
	global wrench, Torque, H, msg_being_published, print_once
	global r_hat_A, Fx, Fy, Fz, Tx, Ty, Tz, theta_A
	if print_once:
		print("Started estimation...")
		print_once = False
	msg_being_published = True
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
	

	# --- Create new frame for publishing theta -----
	br_r = tf.TransformBroadcaster()
	br_r.sendTransform((0, 0, 0), 
		tf.transformations.quaternion_from_euler(0.0, 1.57, 0.0),
		rospy.Time.now(),
		"ft_transform_r",
		"r_gripper_motor_accelerometer_link")
	# ----------------------------------------------
	R_vect.header.frame_id = 'ft_transform_r'
	R_vect.point.x = r_hat_k[0]
	R_vect.point.y = r_hat_k[1]
	R_vect.point.z = r_hat_k[2]
	pub_R.publish(R_vect)

	true_point.header.frame_id = 'ft_transform_r'
	pub_t.publish(true_point)
	
	
	if save_array == True:
		r_hat_A = np.hstack((r_hat_A, r_hat_k[0:]))
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
	# print ('r_hat_k', r_hat_k)
	# print ('z_k', z_k)
	# print ('H', H)
	# print ('A', A)
	# print ('Q', Q)
	# print ('R', R)
	# print ('P_k', P_k)
	# print ('I', I)
	# -------- Prediction Step --------
	r_hat_k = A * r_hat_k # [6x1]
	#print ('Pred r_hat_k', r_hat_k)
	P_k = P_k + Q # [3x3]; A * P_k * A' + Q; A = A' in this case
	#print ('Pred P_K', P_k)
	# -------- Update Step -----------
	K_k = P_k * (H.transpose()) * np.linalg.inv( (H * P_k * (H.transpose()) ) + R ) # [3x3]; PH'(HPH'+R)-1; H'=-H
	#print('K_k', K_k)
	#print('K_k.shape', K_k.shape)
	#print('H * r_hat_k', H * r_hat_k)
	#print('z_k - H * r_hat_k', z_k - H * r_hat_k)
	# print('K_k * ( z_k - H * r_hat_k )', K_k * ( z_k - H * r_hat_k ))
	r_hat_k = r_hat_k + K_k * ( z_k - H * r_hat_k ) # [3x1]
	#print("r_hat_k", r_hat_k)
	#P_k = (I - K_k * H) * P_k # [3x3]
	#print('r_hat_k', r_hat_k)
	return r_hat_k

def publish_theta_callback(event):
	global theta_A, theta_k, msg_being_published
	if msg_being_published == True:
		theta_k = m.atan2(r_hat_k[0], r_hat_k[2])
		if save_array == True:
			theta_A = np.append(theta_A, theta_k)
		pub_theta.publish(theta_k)
		msg_being_published = False
	return


def point_estimation():
	rospy.init_node('point_estimation', anonymous=True)
	rospy.Subscriber("/ft_transformed/rig_arm", WrenchStamped, wrench_callback)
	rospy.Timer(rospy.Duration(0.1), publish_theta_callback)
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
	
	if save_array == True:
		raw_input("\n Press key to continue")
		r_x = np.array(r_hat_A[0,:])
		r_y = np.array(r_hat_A[1,:])
		r_z = np.array(r_hat_A[2,:])

		#print('size(r_x)', np.size(r_x, 1))
		X_axis = np.linspace(0,(np.size(r_x, 1)-1),np.size(r_x, 1))
		#print('len(X_axis)',len(X_axis))
		#print("len(Fx)",len(Fx))

		sampled_X_axis = np.linspace(0,(np.size(theta_A)-1),np.size(theta_A))
		# figure(0)
		# plot(sampled_X_axis, theta_A, 'g'), title('theta'), ylabel('[rad]')
		# figure(1)
		# subplot(311)
		# plot(X_axis, Fx, 'r'), title('Fx'), ylabel('[N]')
		# subplot(312)
		# plot(X_axis, Fy, 'g'), title('Fy'), ylabel('[N]')
		# subplot(313)
		# plot(X_axis, Fz, 'b'), title('Fz'), ylabel('[N]')

		# figure(2)
		# subplot(311)
		# plot(X_axis, Tx, 'r'), title('Tx'), ylabel('[Nm]')
		# subplot(312)
		# plot(X_axis, Ty, 'g'), title('Ty'), ylabel('[Nm]')
		# subplot(313)
		# plot(X_axis, Tz, 'b'), title('Tz'), ylabel('[Nm]')

		# ---- Ground truth creation ----
		show_gnd_truth = True

		gnd_X = np.zeros(len(X_axis))
		gnd_Y = np.zeros(len(X_axis))
		gnd_Z = np.zeros(len(X_axis))
		#gnd_Z = np.ones(len(X_axis))*0
		gnd_X[start_pt:] = X_value
		gnd_Z[start_pt:] = Z_value#0.41
		ymax = 0.8
		ymin = -0.8
		wFig = 8
		hFig = 8.2		

		#print('type(r_y-gnd_X)',type(r_y-gnd_X))
		#print(r_y - gnd_X)
		figure(3, figsize=(wFig,hFig))
		# ---- R_x ----
		subplot(311), ylim((ymin, ymax)), subplots_adjust(left=0.12,bottom=0.06,right=0.93,top=0.94, wspace=0.20, hspace = 0.47)
		plot(X_axis , np.squeeze(r_x), 'r'), title('$\mathbf{\hat{r}_x}$\n',fontsize=20), ylabel('[m]'), xlabel('[ms]')
		if show_gnd_truth == True:
			plot(X_axis, gnd_X, '--k')
			# legend(('X_hat' , 'True X'), 'lower right')
			# subplot(322), ylim((ymin, ymax))
			# plot(X_axis, np.squeeze(abs(r_x - gnd_X)), 'r'), title('Estimation error for R_x')
			# plot(X_axis, gnd_Y, '--k')
		
		# ---- R_y ----
		subplot(312), ylim((ymin, ymax))
		plot(X_axis, np.squeeze(r_y), 'g'), title('$\mathbf{\hat{r}_y}$\n',fontsize=20), ylabel('[m]'), xlabel('[ms]')
		if show_gnd_truth == True:
			plot(X_axis, gnd_Y, '--k')
			# legend(('Y_hat' , 'True Y'), 'lower right')
			# subplot(324), ylim((ymin, ymax))
			# plot(X_axis, np.squeeze(abs(r_y - gnd_Y)), 'g'), title('Estimation error for R_y')
			# plot(X_axis, gnd_Y, '--k')
		
		# ---- R_z ----
		subplot(313), ylim((ymin, ymax))
		plot(X_axis, np.squeeze(r_z), 'b'), title('$\mathbf{\hat{r}_z}$\n',fontsize=20), ylabel('[m]'), xlabel('[ms]')
		if show_gnd_truth == True:
			plot(X_axis, gnd_Z, '--k')
			# legend( ('Z_hat' , 'True Z'), 'lower right')
			# subplot(326), ylim((ymin, ymax))
			# plot(X_axis, np.squeeze(r_z - gnd_Z), 'b'), title('Estimation error for R_z')
			# plot(X_axis, gnd_Y, '--k')
		
		if show_error:
			error_x = np.squeeze((r_x - gnd_X))
			error_y = np.squeeze((r_y - gnd_Y))
			error_z = np.squeeze((r_z - gnd_Z))
			steady_st_x = r_x[0,response_time:]
			steady_st_y = r_y[0,response_time:]
			steady_st_z = r_z[0,response_time:]
			mu_r_x = np.mean(steady_st_x)
			mu_r_y = np.mean(steady_st_y)
			mu_r_z = np.mean(steady_st_z)
			std_r_x = np.std(steady_st_x)
			std_r_y = np.std(steady_st_y)
			std_r_z = np.std(steady_st_z)
			#print("type(mu_r_x)", type(mu_r_x))
			#mean_x = np.mean(error_x)
			#mean_y = np.mean(error_y)
			#mean_z = np.mean(error_z)
			#print('error_x.shape', error_x.shape)
			ymin_error = -0.5
			ymax_error = 0.5
			# figure(4)
			# # --- Error R_x ---
			# subplot(311), ylim((ymin_error, ymax_error)), subplots_adjust(left=0.12,bottom=0.06,right=0.93,top=0.92, wspace=0.20, hspace = 0.33)
			# plot(X_axis, error_x, 'r')#, title('Estimation error R_x, mean(error): %f, mean(est): %f' % (mean_x, mu_r_x))
			# plot(X_axis, gnd_Y, '--k')

			# # --- Error R_y ---
			# subplot(312), ylim((ymin_error, ymax_error))
			# plot(X_axis, error_y, 'g')#, title('Estimation error R_y,  mean(error): %f, mean(est): %f' % (mean_y, mu_r_y))
			# plot(X_axis, gnd_Y, '--k')

			# # --- Error R_z ---
			# subplot(313), ylim((ymin_error, ymax_error))
			# plot(X_axis, error_z, 'b')#, title('Estimation error R_z, mean(error): %f, mean(est): %f' % (mean_z, mu_r_z))
			# plot(X_axis, gnd_Y, '--k')

			print("------ Relevant Values -------")
			print("init_values: ", r_x[0,0],r_y[0,0],r_z[0,0])
			print("wStd: ", wStd)
			print("vStd: ", vStd)
			print("Response time:", response_time)
			print("mu_x: ", mu_r_x, "std_dev_x: ", std_r_x)
			print("mu_y: ", mu_r_y, "std_dev_y: ", std_r_y)
			print("mu_z: ", mu_r_z, "std_dev_z: ", std_r_z)
			# print("error_x: ", mean_x, "mu_x: ", mu_r_x, "std_dev_x: ", std_r_x)
			# print("error_y: ", mean_y, "mu_y: ", mu_r_y, "std_dev_y: ", std_r_y)
			# print("error_z: ", mean_z, "mu_z: ", mu_r_z, "std_dev_z: ", std_r_z)
	


		# theta = np.arctan2(r_x,r_z)
		# true_theta = np.ones(len(X_axis))*0.0

		# figure(4)
		# plot(X_axis, np.squeeze(theta)), title ("Direction estimate in X-Z plane"), ylabel('[rad]')
		# plot(X_axis, true_theta, '--k')
		show()
