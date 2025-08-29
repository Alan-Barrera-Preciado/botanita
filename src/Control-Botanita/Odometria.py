#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
import numpy as np
import tf
import math

rospy.init_node('odometria')
pub = rospy.Publisher('/odom', Odometry, queue_size=10)

r = 0.08    # radio de las ruedas [m]
L = 0.405    # distancia entre ruedas [m]

x, y, theta = 0.0, 0.0, 0.0
prev_time = None
v_f, omega_f = 0.0, 0.0
        
def rpm_callback(msg):
    global x, y, theta, prev_time, v_f, omega_f

    rpm_l, rpm_r = msg.data

    w_l = -rpm_l * 2 * math.pi / 60
    w_r = rpm_r * 2 * math.pi / 60

    v = r * (w_r + w_l) / 2
    omega = r * (w_r - w_l) / L

    current_time = rospy.Time.now()
    if prev_time is None:
        dt = 0.0
    else:
        dt = (current_time - prev_time).to_sec()
    
    alpha = 0.4
    v_f = alpha*v + (1-alpha)*v_f
    omega_f = alpha*omega + (1-alpha)*omega_f
    
    prev_time = current_time
    theta_mid = theta + 0.5*omega_f*dt
    x += v_f * math.cos(theta_mid) * dt
    y += v_f * math.sin(theta_mid) * dt
    theta += omega_f * dt
    print(theta)
    odom_msg = Odometry()
    odom_msg.header.stamp = current_time
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"

    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.position.z = 0.0
    quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    odom_msg.pose.pose.orientation.x = quat[0]
    odom_msg.pose.pose.orientation.y = quat[1]
    odom_msg.pose.pose.orientation.z = quat[2]
    odom_msg.pose.pose.orientation.w = quat[3]

    quat = odom_msg.pose.pose.orientation
(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
    [quat.x, quat.y, quat.z, quat.w]
)

    odom_msg.twist.twist.linear.x = v_f
    odom_msg.twist.twist.angular.z = omega_f

    pub.publish(odom_msg)

rospy.Subscriber('/rpm_medido', Float32MultiArray, rpm_callback)
rospy.spin()
