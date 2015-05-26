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
force = Float32()

# --- Array saving "_A" = variable saved in array
save_arrays = False
theta_hat_A = np.array([])
mag_force_A = np.array([])
change_A = np.array([])
states_A = np.array([0])

# --- Other variables
first_publish = True
first_callback = True
dT = 0.5
counter_theta = 0
sum_theta = 0
prev_theta = 0
prev_force = 0

def r_pose_callback(msg):
	global theta_hat
	global first_publish
	theta_hat = msg
	if first_publish == True:
		first_publish = False
		print("Estimation has started")
		rospy.Timer(rospy.Duration(dT), derivative_callback)
	return

def r_force_callback(msg):
	global force
	force = msg
	return

def derivative_callback(event):
	global first_callback, counter_theta, sum_theta, prev_force, prev_theta
	global states_A
	theta_now = theta_hat.data
	force_now = force.data
	if first_callback == True:
		prev_theta = theta_now
		prev_force = force_now
		first_callback = False
	delta_theta = theta_now - prev_theta
	delta_force = force_now - prev_force
	if abs(delta_theta) < 0.1:
		counter_theta += 1
		sum_theta += theta_now
	else:
		if counter_theta > 4:
			print("state has been added with %d counts" % counter_theta)
			states_A = np.append(states_A, sum_theta/counter_theta)
		counter_theta = 0
		sum_theta = 0
	
		
		
	prev_theta = theta_now
	prev_force = force_now
	return


def change_detection2():

    rospy.init_node('change_detection2', anonymous=True)
    rospy.Subscriber("theta_estimate", Float32, r_pose_callback)
    rospy.Subscriber("mag_force", Float32, r_force_callback)
    #timer_theta = rospy.Timer(rospy.Duration(2), delta_theta_callback, oneshot=True)
    #rospy.Timer(rospy.Duration(1), test_callback)
    print('...Waiting for msgs to be published... \n')
    rospy.spin()
    return

if __name__ == '__main__':

	change_detection2()

	try:
		rate = rospy.Rate(10.0)
		while not rospy.is_shutdown():
			rate.sleep()

	except KeyboardInterrupt:
		pass
	raw_input("Press to see states")
	print("States:", states_A)

	if save_arrays == True:
		raw_input("Press any key if you wish to see plots")

