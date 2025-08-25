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
Vel_Lineal = 0
Vel_Angular = 0

def joy_callback(data):
    global Ref_Izq, Ref_Der, Vel_Lineal, Vel_Angular

    if data.axes[7] > 0.8:
        Vel_Lineal = Vel_Lineal + 5
    if data.axes[7] < -0.8:
        Vel_Lineal = Vel_Lineal - 5
    if data.axes[6] > 0.8:
        Vel_Angular = Vel_Angular + 0.1
    if data.axes[6] < -0.8:
        Vel_Angular = Vel_Angular - 0.1
    if data.buttons[8] > 0.8:
        Vel_Angular = 0
        Vel_Lineal = 0

    Ref_Izq = -max(min((2*Vel_Lineal - Vel_Angular * 42)/(16), 12), -12)
    Ref_Der = max(min((2*Vel_Lineal + Vel_Angular * 42)/(16), 12), -12)

    Ref = Float32MultiArray()
    Ref.data = [Ref_Izq, Ref_Der]
    pub.publish(Ref)

rospy.Subscriber("/joy", Joy, joy_callback)
rospy.spin()
