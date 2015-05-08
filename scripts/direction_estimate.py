#!/usr/bin/python
import roslib
import rospy
import math as m
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Quaternion
import numpy as np
import tf


pub_r = rospy.Publisher("/direction/right_arm", PoseStamped)

R_wrench = WrenchStamped()
L_wrench = WrenchStamped()
R_dir = PoseStamped()
L_dir = PoseStamped()
#quat = Quaternion()

def r_direction_callback(msg):
    R_wrench = msg
    br_r = tf.TransformBroadcaster()
    br_r.sendTransform((0, 0, 0),
         tf.transformations.quaternion_from_euler(0.0, 3.14, -1.57),
         rospy.Time.now(),
         "ft_transform_r",
         "r_gripper_motor_accelerometer_link")
    N_force = m.sqrt((m.pow(R_wrench.wrench.force.x, 2) +
               m.pow(R_wrench.wrench.force.z, 2)))
    theta = m.acos(R_wrench.wrench.force.x / N_force)
##    rospy.loginfo("Fx: %f", R_wrench.wrench.force.x)
##    rospy.loginfo("Fz: %f", R_wrench.wrench.force.z)
##    rospy.loginfo("Check N_force: %f", N_force)
##    rospy.loginfo("Check theta: %f", theta)
    R_dir.header.frame_id = 'ft_transform_r'
    quat = tf.transformations.quaternion_from_euler(0.0, theta, 0.0)
      
    R_dir.pose.orientation.x = quat[0]
    R_dir.pose.orientation.y = quat[1]
    R_dir.pose.orientation.z = quat[2]
    R_dir.pose.orientation.w = quat[3]
    
    pub_r.publish(R_dir)    
    
    return

def l_direction_callback(msg):
##    L_wrench = msg
##    R_pose.header.frame_id = 'l_force_torque_adapter_link'
    return




def direction_estimate():
    rospy.init_node('direction_estimate', anonymous=True)
    rospy.Subscriber("/ft_transformed/rig_arm", WrenchStamped, r_direction_callback)
    rospy.Subscriber("/ft_transformed/lef_arm", WrenchStamped, l_direction_callback)
    
    br_l = tf.TransformBroadcaster()
    rospy.spin()
    return

    
if __name__ == '__main__':

    direction_estimate()

    try:
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():

            br_l.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(1.57, -3.14, 0.0),
                     rospy.Time.now(),
                     "ft_transform_l",
                     "l_gripper_motor_accelerometer_link")
            rate.sleep()

    except KeyboardInterrupt:
        pass

