#!/usr/bin/python
import roslib
import rospy
from matplotlib.pyplot import *
from std_msgs.msg import Float32
import numpy as np
import math as m
import tf

# --- Message Variables
theta_hat = Float32()
mag_force = Float32()

# --- Array saving "_A" = variable saved in array
theta_hat_A = np.array([])
mag_force_A = np.array([])
change_A = np.array([])

# --- Other variables
save_arrays = True
first_time_force = True
first_time_F_changed = True
call_timer = True
prev_force = 0
prev_theta = 0
theta_prev = 0
counter = 0 
force_changed = False
prev_time_f_ch = 0
time_force_changed = 0


# --- Tunable parameters
thresh_d_force = 1
thresh_d_theta = 0.2

def r_pose_callback(msg):
	global	theta_hat, theta_hat_A
	theta_hat = msg
	if save_arrays == True:
		theta_hat_A = np.append(theta_hat_A, theta_hat.data)
	return

def r_force_callback(msg):
	global mag_force, first_time_force, prev_force, mag_force_A, change_A, theta_prev
	global force_changed, prev_time_f_ch, first_time_F_changed, time_force_changed
	mag_force = msg
	if save_arrays == True:
		mag_force_A = np.append(mag_force_A, mag_force.data)
	if first_time_force == True:
		print("Estimation has started to publish")
		prev_force = mag_force.data
		theta_prev = theta_hat.data
		timer = rospy.Timer(rospy.Duration(0.5), delta_F_callback)
		first_time_force = False
	if force_changed == True:
		now = rospy.Time.now()
		dT = now.to_sec() - time_force_changed
		if dT >= 1:
			#print('2 secs passed')
			d_theta = theta_hat.data - prev_theta
			print('prev_theta', prev_theta)
			print('current_theta ', theta_hat.data)
			print('d_theta = %f \n' % d_theta)
			if abs(d_theta) > thresh_d_theta:
				direction = check_direction_theta(d_theta)
				if save_arrays == True:
					change_A = np.append(change_A, direction * abs(d_theta))
			else:
				print('d_theta did not exceed threshold ')
				if save_arrays == True:
					change_A = np.insert(change_A, len(change_A), 0.0)
			force_changed = False
		else:
			if save_arrays == True:
				change_A = np.insert(change_A, len(change_A), 0.0)
	else:
		if save_arrays == True:
			change_A = np.insert(change_A, len(change_A), 0.0)		
	return

def check_direction_theta(d_theta):
	if d_theta < 0:
		direction = -1
		print("       NEG change in theta: moved LEFT a little")
	else:
		direction = 1
		print("       POS change in theta: moved RIGHT a little")
	return direction

def delta_F_callback(event):
	global mag_force, thresh_d_force, prev_force, theta_hat, first_time_force, theta_prev
	global force_changed, prev_time_f_ch, first_time_F_changed, time_force_changed
	global prev_theta
	theta_now = theta_hat.data
	d_force = mag_force.data - prev_force
	prev_force = mag_force.data

	if abs(d_force) > thresh_d_force and force_changed == False:
		if save_arrays == True:
			print("delta force is bigger than threshold at sample: ", len(mag_force_A))
		else:
			print('Change in delta force')
		prev_theta = theta_prev
		now = rospy.Time.now()
		time_force_changed = now.to_sec()
		force_changed = True
	
	theta_prev = theta_now
	
	
	return


def change_detection1():

    rospy.init_node('change_detection1', anonymous=True)
    rospy.Subscriber("theta_estimate", Float32, r_pose_callback)
    rospy.Subscriber("mag_force", Float32, r_force_callback)
    #timer_theta = rospy.Timer(rospy.Duration(2), delta_theta_callback, oneshot=True)
    #rospy.Timer(rospy.Duration(1), test_callback)
    print('...Waiting for msgs to be published... \n')
    rospy.spin()
    return

if __name__ == '__main__':

	change_detection1()

	try:
		rate = rospy.Rate(10.0)
		while not rospy.is_shutdown():
			rate.sleep()

	except KeyboardInterrupt:
		pass
	
	if save_arrays == True:
		raw_input("Press any key if you wish to see plots")
		print('len(mag_force_A)', len(mag_force_A))
		print('len(theta_hat)', len(theta_hat_A))
		print('len(change_A)', len(change_A))
		X_axis = np.linspace(0,(len(mag_force_A)-1),len(mag_force_A))
	
		figure(1)
		subplot(311)
		plot(X_axis, mag_force_A, 'b'), title('mag_force'), ylabel('[N]')
		subplot(312)
		plot(X_axis, theta_hat_A, 'g'), title('theta_hat'), ylabel('[rad]')
		subplot(313)
		plot(X_axis, change_A, 'r'), title('change_A')

		show()
