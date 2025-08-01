#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def joy_callback(data):
    twist = Twist()
    twist.linear.x = data.axes[1]  # Ej: Eje izquierdo vertical (avance/retroceso)
    twist.angular.z = data.axes[0] # Eje izquierdo horizontal (giro)
    pub.publish(twist)

rospy.init_node('xbox_to_twist')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.Subscriber("/joy", Joy, joy_callback)
rospy.spin()
