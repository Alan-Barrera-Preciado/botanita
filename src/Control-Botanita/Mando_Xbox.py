#!/usr/bin/env python3

import rospy
import rospkg
import subprocess
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import time
import os

rospack = rospkg.RosPack()
pkg_path = rospack.get_path("botanita")   # nombre del paquete ROS
Map_dir = os.path.join(pkg_path, "src/Mapas")

rospy.init_node('referencias_motores')
ros_env = os.environ.copy()
ros_env["ROS_MASTER_URI"] = rospy.get_param("/ros_master_uri", "http://localhost:11311")
pub = rospy.Publisher('/vel_referencia', Float32MultiArray, queue_size=10)
status = rospy.Publisher('/robot_status', Float32, queue_size=10)
map = rospy.Publisher('/syscommand', String, queue_size=10)
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
    if data.buttons[3] > 0.8:
        Vel_Angular = 0
        Vel_Lineal = 0
    if data.buttons[2] > 0.8:
        Map_Name = rospy.get_param("~Map_Name", "Mapa")
        map.publish("savegeotiff")
        timestamp = time.strftime("%H%M%S")
        Map_Name_Time = f"{Map_Name}_{timestamp}"
        subprocess.Popen(f"rosrun map_server map_saver -f {Map_dir}/{Map_Name_Time}", shell=True, env=ros_env)

    Ref_Izq = -max(min((2*Vel_Lineal - Vel_Angular * 42)/(16), 12), -12)
    Ref_Der = max(min((2*Vel_Lineal + Vel_Angular * 42)/(16), 12), -12)

    Estatus = Float32()
    Estatus.data = 2.0
    status.publish(Estatus.data)

    Ref = Float32MultiArray()
    Ref.data = [Ref_Izq, Ref_Der]
    pub.publish(Ref)

rospy.Subscriber("/joy", Joy, joy_callback)
rospy.spin()
