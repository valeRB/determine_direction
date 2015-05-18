#!/usr/bin/python
import roslib
import rospy
import math as m
from matplotlib.pyplot import *
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32
import numpy as np
import kalman as K
import tf

pub_r = rospy.Publisher("/direction/right_arm_estimate", PoseStamped)
pub_r_t = rospy.Publisher("/direction/right_arm", PoseStamped)
pub_l = rospy.Publisher("/direction/left_arm", PoseStamped)
pub_theta_hat = rospy.Publisher("/theta_estimate", Float32)

R_wrench = WrenchStamped()
L_wrench = WrenchStamped()
R_dir = PoseStamped()
R_dir_t = PoseStamped()
L_dir = PoseStamped()
theta_pub = Float32()

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
estimation_started = 0
print_once = 0

def r_direction_callback(msg):
    global first_estimate, xhat, P, theta_euler, K_gain, estimation_started, print_once
    global THETA, THETA_hat, THETA_2q, PP, KK_gain, Fx, Fy, Fz, mag_FORCE
    R_wrench = msg
    # --- Collect Fx, Fy and Fz to plot later --
    Fx = np. append(Fx, R_wrench.wrench.force.x)
    Fy = np. append(Fy, R_wrench.wrench.force.y)
    Fz = np. append(Fz, R_wrench.wrench.force.z)
    # -----------------------------------------

    # --- Create new frame for publishing theta -----
    br_r = tf.TransformBroadcaster()
    br_r.sendTransform((0, 0, 0),
         tf.transformations.quaternion_from_euler(0.0, 1.57, 0.0),
         rospy.Time.now(),
         "ft_transform_r",
         "r_gripper_motor_accelerometer_link")
    # ----------------------------------------------
    N_force = m.sqrt((m.pow(R_wrench.wrench.force.x, 2) + m.pow(R_wrench.wrench.force.z, 2)))
    if abs(R_wrench.wrench.force.z) > 0.5 or estimation_started == 1:
        if estimation_started == 0:
            print("Someone started to hold the object!")
        estimation_started = 1

        # mag_FORCE = np.append(mag_FORCE, N_force)
        #theta = m.acos(R_wrench.wrench.force.x / N_force)
        # Fx_n = R_wrench.wrench.force.x/N_force
        # Fy_n = R_wrench.wrench.force.y/N_force
        # theta_n = m.acos(Fx_n)
        theta_r = m.atan2(R_wrench.wrench.force.z, R_wrench.wrench.force.x)
        if theta_r < 0:
            theta_r_2q = theta_r + m.pi
        else:
            theta_r_2q = theta_r
        THETA = np.append(THETA, theta_r)
        THETA_2q = np.append(THETA_2q, theta_r_2q)

        # --- Theta_hat estimation with Kalman Filter ---
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

        # --- Publish Theta for change_detection node ---
        theta_pub = xhat
        pub_theta_hat.publish(theta_pub)
        # ----------------------------------------------

        # --- Publish Theta from Force Measurements ---
        # Note: Theta is published negative for visualization
        R_dir.header.frame_id = 'ft_transform_r'    
        theta_euler = xhat
        quat = tf.transformations.quaternion_from_euler(0.0, -theta_euler, 0.0)
        R_dir.pose.orientation.x = quat[0]
        R_dir.pose.orientation.y = quat[1]
        R_dir.pose.orientation.z = quat[2]
        R_dir.pose.orientation.w = quat[3]
        pub_r.publish(R_dir)
        # ---------------------------------------------

        # --- Publish estimated Theta -----------------
        # Note: Theta is published negative for visualization
        R_dir_t.header.frame_id = 'ft_transform_r'        
        quat = tf.transformations.quaternion_from_euler(0.0, -theta_r, 0.0)
        R_dir_t.pose.orientation.x = quat[0]
        R_dir_t.pose.orientation.y = quat[1]
        R_dir_t.pose.orientation.z = quat[2]
        R_dir_t.pose.orientation.w = quat[3]
        pub_r_t.publish(R_dir_t) 
        # ---------------------------------------------
    else:
        if estimation_started == 0:
            if print_once == 0:
                print("No one is holding the object")
                print_once = 1
        else:
            if print_once == 1:
                print("Someone stopped holding the object! \n Estimation has STOPPED")
                print_once = 2
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
    X_axis_F = np.linspace(0,(len(Fx)-1),len(Fx))
    print('len(X_axis)',len(X_axis))
    print('len(X_axis_F)',len(X_axis_F))
    
    figure(1)
    plot(X_axis, THETA_2q, 'g'), title('THETA_2q'), ylabel('[rad]')
    plot(X_axis, THETA_hat, 'r'), title('THETA_hat'), ylabel('[rad]')
    legend(('THETA_2q', 'Estimated THETA'),'upper right')

    figure(2)
    subplot(311)
    #plot(X_axis, THETA, 'b'), title('THETA'), ylabel('[rad]')
    plot(X_axis, THETA_2q, 'g'), title('THETA_2q'), ylabel('[rad]')
    plot(X_axis, THETA_hat, 'r'), title('THETA_hat'), ylabel('[rad]')
    legend(('THETA_2q', 'Estimated THETA'),'upper right')
    
    subplot(312)
    plot(X_axis_F, Fx, 'r'), title('Fx'), ylabel('[N]')
    subplot(313)
    plot(X_axis_F, Fz, 'b'), title('Fz'), ylabel('[N]')

##    plot(X_axis, mag_FORCE, 'r'), title("Magnitude of Force")

    # figure("Kalman Filter")
    # subplot(211)
    # plot(X_axis, KK_gain),title("Kalman Gain for theta")
    # subplot(212)
    # plot(X_axis, np.sqrt(PP)),title("Covariance for theta")
    
    show()

