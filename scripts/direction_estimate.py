#!/usr/bin/python
import roslib
import rospy
import math as m
from matplotlib.pyplot import *
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Quaternion
import numpy as np
import kalman as K
import tf


pub_r = rospy.Publisher("/direction/right_arm_estimate", PoseStamped)
pub_r_t = rospy.Publisher("/direction/right_arm", PoseStamped)
pub_l = rospy.Publisher("/direction/left_arm", PoseStamped)

R_wrench = WrenchStamped()
L_wrench = WrenchStamped()
R_dir = PoseStamped()
R_dir_t = PoseStamped()
L_dir = PoseStamped()

#--- Arrays to store values
THETA = np.array([])
THETA_2q = np.array([])
THETA_hat = np.array([])
mag_FORCE = np.array([])
KK_gain = np.array([])
PP = np.array([])
Fx = np.array([])
Fy = np.array([])
Fz = np.array([])

#--- Kalman Filter initial values
xhat_0 = 1.57 # start estimate at pi/2 for theta
P_0 = 1
first_estimate = 1
xhat = 0
P = 0
K_gain = 0
theta_euler = 0

def r_direction_callback(msg):
    global first_estimate, xhat, P, theta_euler, K_gain
    global THETA, THETA_hat, THETA_2q, PP, KK_gain, Fx, Fy, Fz, mag_FORCE
    R_wrench = msg
    # --- Collect Fx, Fy and Fz to plot later --
    Fx = np. append(Fx, R_wrench.wrench.force.x)
    Fy = np. append(Fy, R_wrench.wrench.force.y)
    Fz = np. append(Fz, R_wrench.wrench.force.z)
    # -----------------------------------------
    br_r = tf.TransformBroadcaster()
    br_r.sendTransform((0, 0, 0),
         tf.transformations.quaternion_from_euler(0.0, 1.57, 0.0),
         rospy.Time.now(),
         "ft_transform_r",
         "r_gripper_motor_accelerometer_link")
    
    N_force = m.sqrt((m.pow(R_wrench.wrench.force.x, 2) +
               m.pow(R_wrench.wrench.force.z, 2)))
    mag_FORCE = np.append(mag_FORCE, N_force)
##    theta = m.acos(R_wrench.wrench.force.x / N_force)
    Fx_n = R_wrench.wrench.force.x/N_force
    Fy_n = R_wrench.wrench.force.y/N_force
    theta_n = m.acos(Fx_n)
    theta_r = m.atan2(R_wrench.wrench.force.z, R_wrench.wrench.force.x)
    if theta_r < 0:
        theta_r_2q = theta_r + m.pi
    else:
        theta_r_2q = theta_r
##    print('Fx: ', R_wrench.wrench.force.x)
##    print('Fy: ', R_wrench.wrench.force.z)
##    print('Fx_n', Fx_n)
##    print('N_force', N_force)
##    print('theta_n = m.acos(Fx_n)', theta_n)
##    print('theta_r = atan2...', theta_r)
##    print('theta_r_2q', theta_r_2q)
    THETA = np.append(THETA, theta_r)
    THETA_2q = np.append(THETA_2q, theta_r_2q)
    if first_estimate == 1:
        (xhat, P, K_gain) = K.Kalman_Filter(theta_r_2q, xhat_0, P_0)
        KK_gain = np.append(KK_gain, K_gain)
        PP = np.append(PP, P)
        THETA_hat = np.append(THETA_hat, xhat)
        first_estimate = 0
        
    else:
        (xhat, P, K_gain) = K.Kalman_Filter(theta_r_2q, xhat, P)
        KK_gain = np.append(KK_gain, K_gain)
        PP = np.append(PP, P)
        THETA_hat = np.append(THETA_hat, xhat)
    
    ## -- 0 deg= 0 rad; 90 = pi/2; 180 = pi; 270 = -pi/2

    R_dir.header.frame_id = 'ft_transform_r'    
    theta_euler = xhat
    quat = tf.transformations.quaternion_from_euler(0.0, -theta_euler, 0.0)
    
    R_dir.pose.orientation.x = quat[0]
    R_dir.pose.orientation.y = quat[1]
    R_dir.pose.orientation.z = quat[2]
    R_dir.pose.orientation.w = quat[3]

    pub_r.publish(R_dir)

    # ______ Publish Actual Theta ______
    R_dir_t.header.frame_id = 'ft_transform_r'    
    
    quat = tf.transformations.quaternion_from_euler(0.0, -theta_r, 0.0)
    
    R_dir_t.pose.orientation.x = quat[0]
    R_dir_t.pose.orientation.y = quat[1]
    R_dir_t.pose.orientation.z = quat[2]
    R_dir_t.pose.orientation.w = quat[3]

    pub_r.publish(R_dir)
    pub_r_t.publish(R_dir_t) 
    
    return

def l_direction_callback(msg):
    L_wrench = msg
    br_l = tf.TransformBroadcaster()
    br_l.sendTransform((0, 0, 0),
         tf.transformations.quaternion_from_euler(0.0, 1.57, 0.0),
         rospy.Time.now(),
         "ft_transform_l",
         "l_gripper_motor_accelerometer_link")
    theta_l = m.atan2(L_wrench.wrench.force.z, L_wrench.wrench.force.x)
    ## -- 0 deg= 0 rad; 90 = pi/2; 180 = pi; 270 = -pi/2
##    if theta_l > 0:
##        theta_r += m.pi
##    rospy.loginfo("Theta: %f", theta_r)
    L_dir.header.frame_id = 'ft_transform_l'
    quat_l = tf.transformations.quaternion_from_euler(0.0, theta_l, 0.0)
      
    L_dir.pose.orientation.x = quat_l[0]
    L_dir.pose.orientation.y = quat_l[1]
    L_dir.pose.orientation.z = quat_l[2]
    L_dir.pose.orientation.w = quat_l[3]
    
    pub_l.publish(L_dir)    
    
    return


def direction_estimate():
    rospy.init_node('direction_estimate', anonymous=True)
    rospy.Subscriber("/ft_transformed/rig_arm", WrenchStamped, r_direction_callback)
    #rospy.Subscriber("/ft_transformed/lef_arm", WrenchStamped, l_direction_callback)
    
    rospy.spin()
    return

    
if __name__ == '__main__':

    direction_estimate()

    try:
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():

            rate.sleep()

    except KeyboardInterrupt:
        pass
    raw_input("Press any key to see plot of theta")
    X_axis = np.linspace(0,(len(THETA)-1),len(THETA))
    print('len(THETA_hat)', len(THETA_hat))
    print('len(X_axis)',len(X_axis))
    figure("Theta and theta_hat")
    subplot(311)
    #plot(X_axis, THETA, 'b'), title('THETA'), ylabel('[rad]')
    plot(X_axis, THETA_2q, 'g'), title('THETA_2q'), ylabel('[rad]')
    plot(X_axis, THETA_hat, 'r'), title('THETA_hat'), ylabel('[rad]')
    legend(('THETA_2q', 'Estimated THETA'),'upper right')
    subplot(312)
    plot(X_axis, Fx, 'r'), title('Fx'), ylabel('[N]')
    subplot(313)
    plot(X_axis, Fz, 'b'), title('Fz'), ylabel('[N]')

##    plot(X_axis, mag_FORCE, 'r'), title("Magnitude of Force")

    figure("Kalman Filter")
    subplot(211)
    plot(X_axis, KK_gain),title("Kalman Gain for theta")
    subplot(212)
    plot(X_axis, np.sqrt(PP)),title("Covariance for theta")

##    
##    figure("Analyzing magnitude of force")
##    subplot(311)
##    plot(X_axis, mag_FORCE), title('magnitude of force' )
##
##    diff_F = np.diff(mag_FORCE)
##    delta = 50
##    diff_F_delta = np.array([])
##    multiplos_delta = len(mag_FORCE) - (len(mag_FORCE) % delta)
##    
##    for i in range(0, (multiplos_delta- 1 - delta)):        
##        dif_delta = mag_FORCE[i+delta] - mag_FORCE[i]
##        diff_F_delta = np.append(diff_F_delta, dif_delta)
##
##    if len(diff_F_delta) < len(X_axis):
##        for i in range(len(diff_F_delta),len(X_axis)):
##            diff_F_delta = np.append(diff_F_delta, 0)    
##    subplot(312)
##    plot(X_axis, diff_F_delta), title('Differentiation of force' )
##
##    subplot(313)
##    plot(X_axis, THETA_hat, 'r'), title('THETA_hat')
    
    
    
    show()

