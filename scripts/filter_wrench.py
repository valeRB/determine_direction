#!/usr/bin/python
import roslib
import rospy
from matplotlib.pyplot import *
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PointStamped
import numpy as np
import math as m
import tf
import kalman as KF

save_array = True
tq_x_prev = 0
tq_y_prev = 0
tq_z_prev = 0
P_prev_x = 0
P_prev_y = 0
P_prev_z = 0
filter_torque = PointStamped()
torque_X = np.array([])
torque_Y = np.array([])
torque_Z = np.array([])
torque_X_raw = np.array([])
torque_Y_raw = np.array([])
torque_Z_raw = np.array([])

pub_point = rospy.Publisher("/filtered_torque", PointStamped)

def wrench_callback(msg):
	global tq_x_prev, tq_y_prev, tq_z_prev, P_prev_x, P_prev_y, P_prev_z
	global torque_X, torque_Y, torque_Z, filter_torque
	global torque_X_raw, torque_Y_raw, torque_Z_raw
	wrench_t = msg
	(tq_x, P_x) = KF.Kalman_Filter(wrench_t.wrench.torque.x, tq_x_prev, P_prev_x)
	(tq_y, P_y) = KF.Kalman_Filter(wrench_t.wrench.torque.y, tq_y_prev, P_prev_y)
	(tq_z, P_z) = KF.Kalman_Filter(wrench_t.wrench.torque.z, tq_z_prev, P_prev_z)
	P_prev_x = P_x
	P_prev_y = P_y
	P_prev_z = P_z
	tq_x_prev = tq_x
	tq_y_prev = tq_y
	tq_z_prev = tq_z
	filter_torque.point.x = tq_x
	filter_torque.point.y = tq_y
	filter_torque.point.z = tq_z
	pub_point.publish(filter_torque)
	if save_array:
		torque_X_raw = np.append(torque_X_raw, wrench_t.wrench.torque.x)
		torque_Y_raw = np.append(torque_Y_raw, wrench_t.wrench.torque.y)
		torque_Z_raw = np.append(torque_Z_raw, wrench_t.wrench.torque.z)
		torque_X = np.append(torque_X, tq_x)
		torque_Y = np.append(torque_Y, tq_y)
		torque_Z = np.append(torque_Z, tq_z)
	return

def filter_wrench():
	rospy.init_node('filter_wrench', anonymous=True)
	rospy.Subscriber("/ft_transformed/rig_arm", WrenchStamped, wrench_callback)
	rospy.spin()
	return

if __name__ == '__main__':

	filter_wrench()

	try:
		rate = rospy.Rate(10.0)
		while not rospy.is_shutdown():
			rate.sleep()

	except KeyboardInterrupt:
		pass
	if save_array == True:
		raw_input("Press any key to see plot of theta")
		zero_vector = np.zeros(len(torque_X_raw))
		X_axis = np.linspace(0,(len(torque_X_raw)-1),len(torque_X_raw))
		figure(1)
		# ---- Raw Torques ----
		subplot(311)#, ylim((ymin, ymax)) #, subplots_adjust(left=0.08,bottom=0.06,right=0.93,top=0.92, wspace=0.20, hspace = 0.33)
		plot(X_axis , torque_X_raw, 'r'), title('Raw T_x'), ylabel('[Nm]')
		plot(X_axis, zero_vector, '--k')
		subplot(312)
		plot(X_axis , torque_Y_raw, 'g'), title('Raw T_y'), ylabel('[Nm]')
		plot(X_axis, zero_vector, '--k')
		subplot(313)
		plot(X_axis , torque_Z_raw, 'b'), title('Raw tq_z'), ylabel('[Nm]')
		plot(X_axis, zero_vector, '--k')

		figure(2)
		# ---- R_x ----
		subplot(311)#, ylim((ymin, ymax)) #, subplots_adjust(left=0.08,bottom=0.06,right=0.93,top=0.92, wspace=0.20, hspace = 0.33)
		plot(X_axis , torque_X, 'r'), title('Filtered T_x'), ylabel('[Nm]')
		plot(X_axis, zero_vector, '--k')
		subplot(312)
		plot(X_axis , torque_Y, 'g'), title('Filtered T_y'), ylabel('[Nm]')
		plot(X_axis, zero_vector, '--k')
		subplot(313)
		plot(X_axis , torque_Z, 'b'), title('Filtered T_z'), ylabel('[Nm]')
		plot(X_axis, zero_vector, '--k')

		show()