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

# --- Other variables
save_arrays = True
first_time = True
call_timer = True
prev_force = 0
prev_theta = 0
counter = 0 



# --- Tunable parameters
thresh_d_force = 1.5

def r_pose_callback(msg):
	global	theta_hat
	theta_hat = msg
	if save_arrays == True:
		theta_hat_A = np.append(theta_hat_A, theta_hat.data)
	return

def r_force_callback(msg):
	global mag_force, first_time, prev_force
	mag_force = msg
	if save_arrays == True:
		mag_force_A = np.append(mag_force_A, mag_force.data)
	if first_time == True:
		prev_force = mag_force.data
		rospy.Timer(rospy.Duration(1), delta_F_callback)
		first_time = False
	return

# def delta_F_callback(event):
# 	global mag_force, thresh_d_force, prev_force, prev_theta, theta_hat
# 	d_force = mag_force.data - prev_force
# 	prev_force = mag_force.data
# 	if abs(d_force) > thresh_d_force:
# 		print("delta force is bigger than threshold")
# 		prev_theta = theta_hat.data
# 		rospy.Timer(rospy.Duration(2), delta_theta_callback)
# 	return

def delta_theta_callback(event):
	global counter
	counter += 1
	print ("theta callback counter", counter)
	if counter > 2:
		print 'stop function'
		change_detection1().timer_theta.shutdown()
	return


def test_callback(event):
 	global counter
 	if counter == 2:
 		print("counter = 2")
		
		counter = 3
 	return




def change_detection1():

    rospy.init_node('change_detection1', anonymous=True)
    rospy.Subscriber("theta_estimate", Float32, r_pose_callback)
    rospy.Subscriber("mag_force", Float32, r_force_callback)
    timer_theta = rospy.Timer(rospy.Duration(2), delta_theta_callback, oneshot=True)

    rospy.Timer(rospy.Duration(1), test_callback)
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