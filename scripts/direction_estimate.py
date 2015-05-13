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

xhat_0 = np.matrix('1.57 ; 0.0') # start estimate at pi/2 for theta
P_0 = np.matrix(np.identity(2))
first_estimate = 1
THETA = np.array([])
THETA_2q = np.array([])
THETA_hat = np.array([])
mag_FORCE = np.array([])
xhat = np.matrix('1.57 ; 0.0')
P = np.matrix([])
theta_euler = 0

def r_direction_callback(msg):
    global THETA, THETA_hat, first_estimate, xhat, P, THETA_2q, mag_FORCE, theta_euler
    R_wrench = msg
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
    theta_r = m.atan2(R_wrench.wrench.force.z, R_wrench.wrench.force.x)
    if theta_r < 0:
        theta_r_2q = theta_r + m.pi
    else:
        theta_r_2q = theta_r
    THETA = np.append(THETA, theta_r)
    THETA_2q = np.append(THETA_2q, theta_r_2q)
    if first_estimate == 1:
        (xhat, P) = K.Kalman_Filter(theta_r_2q, xhat_0, P_0)
        THETA_hat = np.append(THETA_hat, xhat[0])
        first_estimate = 0
        print ("xhat is: "  , xhat[0])
        
    else:
        (xhat, P) = K.Kalman_Filter(theta_r_2q, xhat, P)
        THETA_hat = np.append(THETA_hat, xhat[0])
    
    ## -- 0 deg= 0 rad; 90 = pi/2; 180 = pi; 270 = -pi/2
##    rospy.loginfo("Theta: %f", theta_r)
##    print("45 deg", m.atan2(1,-1))
##    print("225 deg", m.atan2(-1,1))
##    rospy.loginfo("Fz: %f", R_wrench.wrench.force.z)
##    rospy.loginfo("Check N_force: %f", N_force)
##    rospy.loginfo("Check theta: %f", theta_r)
# ______ Publish Estimated Theta ______
    R_dir.header.frame_id = 'ft_transform_r'    
    theta_euler = float(xhat[0])
    quat = tf.transformations.quaternion_from_euler(0.0, theta_euler, 0.0)
    
    R_dir.pose.orientation.x = quat[0]
    R_dir.pose.orientation.y = quat[1]
    R_dir.pose.orientation.z = quat[2]
    R_dir.pose.orientation.w = quat[3]

    pub_r.publish(R_dir)

    # ______ Publish Actual Theta ______
    R_dir_t.header.frame_id = 'ft_transform_r'    
    
    quat = tf.transformations.quaternion_from_euler(0.0, theta_r, 0.0)
    
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
    print('len(THETA)', len(THETA))
##    print('len(THETA_hat)', len(THETA_hat))
    print('len(THETA_2q)', len(THETA_2q))
    print('len(X_axis)',len(X_axis))
    figure("Theta and theta_hat")
    subplot(311)
    plot(X_axis, THETA, 'g'), title('THETA'), ylabel('[rad]')
    subplot(312)
    plot(X_axis, THETA_2q, 'b'), title('THETA_2q')
    plot(X_axis, THETA_hat, 'r'), title('THETA_hat')
    legend(('Theta_2q','THETA_hat'),'upper right')
    subplot(313)
    plot(X_axis, (THETA_2q - THETA_hat), 'r'), title("Error")
    
##    legend(('Theta','Theta_hat'),'upper right')
##
    figure("Analyzing magnitude of force")
    subplot(311)
    plot(X_axis, mag_FORCE), title('magnitude of force' )

    diff_F = np.diff(mag_FORCE)
    delta = 50
    diff_F_delta = np.array([])
    multiplos_delta = len(mag_FORCE) - (len(mag_FORCE) % delta)
    
    for i in range(0, (multiplos_delta- 1 - delta)):        
        dif_delta = mag_FORCE[i+delta] - mag_FORCE[i]
        diff_F_delta = np.append(diff_F_delta, dif_delta)

    if len(diff_F_delta) < len(X_axis):
        for i in range(len(diff_F_delta),len(X_axis)):
            diff_F_delta = np.append(diff_F_delta, 0)
    
    
    subplot(312)
    plot(X_axis, diff_F_delta), title('Differentiation of force' )

    subplot(313)
    plot(X_axis, THETA_hat, 'r'), title('THETA_hat')
    
    
    
    show()

