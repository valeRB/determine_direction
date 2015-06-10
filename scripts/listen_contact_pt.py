#!/usr/bin/python
import roslib
import rospy
from matplotlib.pyplot import *
from geometry_msgs.msg import PointStamped
import numpy as np
import math as m
import tf

r_x = np.array([])
r_y = np.array([])
r_z = np.array([])
publish_point = rospy.Publisher("/adaptive_ctrl/point_estimate", PointStamped)

def point_callback(msg):
	global r_x, r_y, r_z
	ct_point = msg
	r_x = np.append(r_x, ct_point.point.x)
	r_y = np.append(r_y, ct_point.point.y)
	r_z = np.append(r_z, ct_point.point.z)
	# --- Create new frame for publishing theta -----
	br_r = tf.TransformBroadcaster()
	br_r.sendTransform((0, 0, 0), 
		tf.transformations.quaternion_from_euler(0.0, 1.57, 0.0),
		rospy.Time.now(),
		"ft_transform_r",
		"r_gripper_motor_accelerometer_link")
	# ----------------------------------------------
	ct_point.header.frame_id = 'ft_transform_r'
	publish_point.publish(ct_point)
	return

def listen_contact_pt():
	rospy.init_node('listen_contact_pt', anonymous=True)
	rospy.Subscriber("/contact_point_estimation/contact_point_estimate", PointStamped, point_callback)
	rospy.spin()
	return

if __name__ == '__main__':

	listen_contact_pt()

	try:
		rate = rospy.Rate(10.0)
		while not rospy.is_shutdown():
			rate.sleep()

	except KeyboardInterrupt:
		pass
	raw_input("Press enter to see plots")
	X_axis = np.linspace(0,len(r_x)-1,len(r_x))
	# ---- Ground truth creation ----
	start_pt = 0#13000 
	X_value = 0.365  # [m]
	Z_value = 0.64 # [m]
	gnd_X = np.zeros(len(X_axis))
	gnd_Y = np.zeros(len(X_axis))
	#gnd_Z = np.zeros(len(X_axis))
	gnd_Z = np.ones(len(X_axis))*0
	gnd_X[start_pt:] = X_value
	gnd_Z[start_pt:] = Z_value#0.41
	ymax = 0.9
	ymin = -0.5

	figure(1)

	# ---- R_x ----
	subplot(311), ylim((ymin, ymax)) #, subplots_adjust(left=0.08,bottom=0.06,right=0.93,top=0.92, wspace=0.20, hspace = 0.33)
	plot(X_axis , r_x, 'r'), title('Estimate of R_x'), ylabel('[m]')
	plot(X_axis, gnd_X, '--k')
	# legend(('X_hat' , 'True X'), 'lower right')
	subplot(312), ylim((ymin, ymax))
	plot(X_axis , r_y, 'g'), title('Estimate of R_y'), ylabel('[m]')
	plot(X_axis, gnd_Y, '--k')
	# legend(('X_hat' , 'True Y'), 'lower right')
	subplot(313), ylim((ymin, ymax))
	plot(X_axis , r_z, 'b'), title('Estimate of R_z'), ylabel('[m]')
	plot(X_axis, gnd_Z, '--k')
	# legend(('X_hat' , 'True Z'), 'lower right')
	show()