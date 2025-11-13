#!/usr/bin/env python3

# -------- Librerias ----------

import rospy
from std_msgs.msg import Float32MultiArray, Float32
from nav_msgs.msg import Odometry
import tf
import math

# -------- Nodo de ROS y publicadores ----------

rospy.init_node('odometria')
pub = rospy.Publisher('/odom_estimada', Odometry, queue_size=10)
theta_pub = rospy.Publisher('/theta_estimada', Float32, queue_size=10)   # orientación en grados

# -------- Parametros iniciales ----------

r = 0.08    # Radio de las ruedas [m]
L = 0.405   # Distancia entre ruedas [m]
w_l = 0.0
w_r = 0.0

x, y, theta = 0.0, 0.0, 0.0
prev_time = None
v_f, omega_f = 0.0, 0.0

# -------- Recibir mensaje de ROS del topic al que se esta suscrito ----------

def rads_callback(msg):
    global x, y, theta, prev_time, v_f, omega_f

    w_l, w_r = msg.data
    w_l = w_l * -1

    v = r * (w_r + w_l) / 2
    omega = r * (w_r - w_l) / L

    current_time = rospy.Time.now()
    dt = 0.01
    prev_time = current_time

# -------- Filtro exponencial ----------

    alpha = 0.4
    v_f = alpha*v + (1-alpha)*v_f
    omega_f = alpha*omega + (1-alpha)*omega_f

# -------- Integracion ----------

    theta_mid = theta + 0.5*omega_f*dt
    x += v_f * math.cos(theta_mid) * dt
    y += v_f * math.sin(theta_mid) * dt
    theta += omega_f * dt

# -------- Construccion de mensaje de odometria ----------

    odom_msg = Odometry()
    odom_msg.header.stamp = current_time
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"

    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    odom_msg.pose.pose.orientation.x = quat[0]
    odom_msg.pose.pose.orientation.y = quat[1]
    odom_msg.pose.pose.orientation.z = quat[2]
    odom_msg.pose.pose.orientation.w = quat[3]

    odom_msg.twist.twist.linear.x = v_f
    odom_msg.twist.twist.angular.z = omega_f

    pub.publish(odom_msg)

# -------- Publicar Theta en grados en topic diferente ----------

    theta_deg = math.degrees(theta)
    theta_pub.publish(theta_deg)

# -------- Suscripcion de ROS ----------

rospy.Subscriber('/rads_motores', Float32MultiArray, rads_callback)
rospy.spin()

