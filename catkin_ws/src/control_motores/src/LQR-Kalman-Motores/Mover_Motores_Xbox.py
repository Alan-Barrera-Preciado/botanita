#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

rospy.init_node('referencias_motores')
pub = rospy.Publisher('/vel_referencia',
 Float32MultiArray, queue_size=10)
rate = rospy.Rate(20)  # 50 ms
Ref_Izq = 0
Ref_Der = 0

def joy_callback(data):
    global Ref_Izq, Ref_Der
    if data.axes[7] > 0.8:
        Ref_Izq = max(Ref_Izq - 0.5, -15)
    if data.axes[7] < -0.8:
        Ref_Izq = min(Ref_Izq + 0.5, 15)
    if data.buttons[3] > 0.8:
        Ref_Der = min(Ref_Der + 0.5, 15)
    if data.buttons[0] > 0.8:
        Ref_Der = max(Ref_Der - 0.5, -15)
    if data.buttons[8] > 0.8:
        Ref_Izq = 0
        Ref_Der = 0

    Ref = Float32MultiArray()
    Ref.data = [Ref_Izq, Ref_Der]
    pub.publish(Ref)

rospy.Subscriber("/joy", Joy, joy_callback)
rospy.spin()
