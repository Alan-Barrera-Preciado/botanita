#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import FLoat32MultiArray

rospy.init_node('referencias_motores')
pub = rospy.Publisher('/vel_referencia',
 Float32MultiArray, queue_size=10)
rospy.Subscriber("/joy", Joy, joy_callback)
rate = rospy.Rate(20)  # 50 ms
Ref_Izq = 0
Ref_Der = 0

def joy_callback(data):
    
    if data.axes[7] > 0.8
        Ref_Izq = Ref_Izq + 0.5
    if data.axes[7] < -0.8
        Ref_Izq = Ref_Izq - 0.5
    if data.buttons[3] > 0.8
        Ref_Der = Ref_Der + 0.5
    if data.buttons[0] > 0.8
        Ref_Der = Ref_Der - 0.5
    if data.buttons[8] > 0.8
        Ref_Izq = 0
        Ref_Der = 0
    
    Ref = Float32MultiArray()
    Ref.data = [Ref_Izq, Ref_Der]
    pub.publish(Ref)
    rate.sleep()

rospy.spin()
