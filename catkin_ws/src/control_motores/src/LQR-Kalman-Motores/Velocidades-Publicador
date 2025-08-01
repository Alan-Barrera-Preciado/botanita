#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

rospy.init_node('referencias_motores')
pub = rospy.Publisher('/vel_referencia', Float32MultiArray, queue_size=10)
rate = rospy.Rate(20)  # 50 ms

t = 0
while not rospy.is_shutdown():
    Ref = Float32MultiArray()
    Ref_Izq = 5
    Ref_Der = 5
    Ref.data = [Ref_Izq, Ref_Der]
    pub.publish(Ref)
    t += 0.05
    rate.sleep()

