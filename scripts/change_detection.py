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
mag_Force = Float32()

pose_is_published = False
thresh_diff_theta = 0.2
thresh_diff_force = 1.5
theta_0 = m.pi/2
check_next_dTheta = False
first_iteration = True

# --- Arrays for plotting ---
theta_2s = np.array([])
delta_theta = np.array([])
d_theta_thres = np.array([])
d_force_thres = np.array([])
m_force_2s = np.array([])
delta_force = np.array([])

# ---------------------------

def r_pose_callback(msg):
	global	theta_hat, pose_is_published
	pose_is_published = True
	theta_hat = msg
	return

def r_force_callback(msg):
	global mag_Force
	mag_Force = msg
	return



def change_callback(event):
	global pose_is_published, theta_hat, theta_0, check_next_dTheta, mag_Force, first_iteration, force_0
	global theta_2s, delta_theta, d_theta_thres, m_force_2s, delta_force, d_force_thres
	if pose_is_published == False:
		print("No pose being published")
	else: 
		# print('theta_hat', theta_hat.data)
		#theta_2s = np.append(theta_2s, theta_hat.data)
		m_force_2s = np.append(m_force_2s, mag_Force.data)
		#diff_theta = theta_hat.data - theta_0
		if first_iteration == True:
			force_0 = mag_Force.data
			first_iteration = False
		diff_force = mag_Force.data - force_0
		delta_theta = np.append(delta_theta, diff_theta)
		delta_force = np.append(delta_force, diff_force)
		theta_0 = theta_hat.data
		force_0 = mag_Force.data

		if abs(diff_theta) > thresh_diff_theta:
			print("dTheta bigger than threshold")
			if abs(diff_force) > thresh_diff_force:
				print("   dForce bigger than threshold")
				if diff_theta < 0:
					d_theta_thres = np.append(d_theta_thres, -1)
					print("       NEG change in theta: moved LEFT a little")
				else:
					d_theta_thres = np.append(d_theta_thres, 1)	
					print("       POS change in theta: moved RIGHT a little")
				if diff_force < 0:
					d_force_thres = np.append(d_force_thres, -1)
					print("       NEG change in force")
				else:
					d_force_thres = np.append(d_force_thres, 1)
					print("       POS change in force")
					
			else:
				print("   dForce is NOT indicating a big change")
				print("   current dForce = %f", diff_force)
				print("   previous dForce = %f", delta_force[-2])
				d_force_thres = np.insert(d_force_thres, len(d_force_thres), 0.0)
		 		d_theta_thres = np.insert(d_theta_thres, len(d_theta_thres), 0.0)
		else:
			d_force_thres = np.insert(d_force_thres, len(d_force_thres), 0.0)
		 	d_theta_thres = np.insert(d_theta_thres, len(d_theta_thres), 0.0)			

		

		# # --- Threshold for diff_force ----------
		# if abs(diff_force) > thresh_diff_force:
		# 	print("Force bigger than threshold")
		# 	# --- Check direction of change in theta:
		# 	if diff_theta < 0:
		# 		d_theta_thres = np.append(d_theta_thres, -1)
		# 		print("   NEG change in theta: moved LEFT a little")
		# 	else:
		# 		d_theta_thres = np.append(d_theta_thres, 1)	
		# 		print("   POS change in theta: moved RIGHT a little")
		# 	# --- Chech direction of change in ||F||
		# 	if diff_force < 0:
		# 		d_force_thres = np.append(d_force_thres, -1)
		# 		print("      NEG change in force")
		# 	else:
		# 		d_force_thres = np.append(d_force_thres, 1)
		# 		print("      POS change in force")
		# else:
		# 	d_force_thres = np.insert(d_force_thres, len(d_force_thres), 0.0)
		# 	d_theta_thres = np.insert(d_theta_thres, len(d_theta_thres), 0.0)


		#print('diff_after_thresh', diff_after_thresh)


	pose_is_published = False
	return


def change_detection():
    rospy.init_node('change_detection', anonymous=True)
    rospy.Subscriber("theta_estimate", Float32, r_pose_callback)
    rospy.Subscriber("mag_force", Float32, r_force_callback)
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
    print('len(d_theta_thres)', len(d_theta_thres))
    
    figure(1)
    subplot(211)
    plot(X_axis, theta_2s, 'b'), title('theta'), ylabel('[rad]')
    subplot(212)
    plot(X_axis, m_force_2s, 'b'), title('Force'), ylabel('[N]')

    figure(2)
    subplot(211)
    plot(X_axis, delta_force, 'b'), title('Delta force'), ylabel('[N]')
    subplot(212)
    plot(X_axis, delta_theta, 'g'), title('delta_theta; dT = 2s')
    
    figure(3)
    subplot(211)
    plot(X_axis, d_force_thres, 'b'), title('Delta force after threshold; thresh = %f' % thresh_diff_force)
    subplot(212)
    plot(X_axis, d_theta_thres, 'g'), title('Delta theta after change in force is significant')


    show()
