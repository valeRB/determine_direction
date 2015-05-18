#!/usr/bin/python
import roslib
import rospy
from matplotlib.pyplot import *
from std_msgs.msg import Float32
import numpy as np
import kalman as K
import math as m
import tf

# --- Message Variables
theta_hat = Float32()

pose_is_published = False
thresh_diff = 0.2
theta_0 = m.pi/2
change_counter = 0

# --- Arrays for plotting ---
theta_2s = np.array([])
delta_theta = np.array([])
diff_after_thresh = np.array([])
# ---------------------------

def r_pose_callback(msg):
	global	theta_hat, pose_is_published
	pose_is_published = True
	theta_hat = msg
	return

def change_callback(event):
	global pose_is_published, theta_hat, theta_0, change_counter
	global theta_2s, delta_theta, diff_after_thresh
	if pose_is_published == False:
		print("No pose being published")
	else:
		# print('theta_hat', theta_hat.data)
		theta_2s = np.append(theta_2s, theta_hat.data)
		diff = theta_hat.data - theta_0
		delta_theta = np.append(delta_theta, diff)
		if abs(diff) > thresh_diff:
			if diff < 0:
				diff_after_thresh = np.append(diff_after_thresh, -1)
				change_counter += 1
			else:
				diff_after_thresh = np.append(diff_after_thresh, 1)
				change_counter += 1
			print('A change occurred; total changes = ', change_counter)
		else:
			# print('no big changes')
			diff_after_thresh = np.insert(diff_after_thresh, len(diff_after_thresh), 0.0)

		#print('diff_after_thresh', diff_after_thresh)
		theta_0 = theta_hat.data

	pose_is_published = False
	return


def change_detection():
    rospy.init_node('change_detection', anonymous=True)
    rospy.Subscriber("theta_estimate", Float32, r_pose_callback)
    rospy.Timer(rospy.Duration(2), change_callback)
    rospy.spin()
    return

if __name__ == '__main__':

    change_detection()

    try:
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            rate.sleep()

    except KeyboardInterrupt:
        pass
    print('Plotting info')
    print('len(theta_2s)', len(theta_2s))
    X_axis = np.linspace(0,(len(theta_2s)-1),len(theta_2s))
    print('len(X_axis)', len(X_axis))
    print('len(diff_after_thresh)', len(diff_after_thresh))
    
    figure(1)
    subplot(311)
    plot(X_axis, theta_2s, 'b'), title('theta'), ylabel('[rad]')
    subplot(312)
    plot(X_axis, delta_theta, 'b'), title('delta_theta; dT = 2s')
    subplot(313)
    plot(X_axis, diff_after_thresh, 'g'), title('delta_theta after threshold; thresh = %f' % thresh_diff)

    show()
