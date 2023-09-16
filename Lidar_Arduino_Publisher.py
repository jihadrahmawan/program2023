#!/usr/bin/env python



import rospy
import serial
from std_msgs.msg import Int32MultiArray

# Arduino connection 
connection_bus='/dev/ttyUSB0'
baud=57600
msgs = Int32MultiArray()

def arduino_read(strings):
    try:
        dpn = int (strings[0:4])
    except ValueError:
        dpn = 0
    try:
        bwh = int  (strings[4:8])
    except ValueError:
        bwh = 0
    try :
        kan = int  (strings[8:12])
    except ValueError:
        kan = 0
    try:
        kir = int  (strings[12:16])
    except ValueError:
        kir = 0

    return dpn, bwh, kan, kir


if __name__=="__main__":

    rospy.init_node('lidar_data')
    arduinoLidar = rospy.Publisher('arduinoLidar_pub', Int32MultiArray, queue_size=1)
    arduino_data = serial.Serial(connection_bus,baud, timeout=1)
    
    r = rospy.Rate(30) # 10hz

    while not rospy.is_shutdown():
        lidar_Depan, lidar_bawah, lidar_kanan, lidar_kiri = arduino_read(arduino_data.readline())
        msgs.data = [lidar_Depan, lidar_bawah, lidar_kanan, lidar_kiri]
        print("[INFO] INFO DATA SEND", msgs.data)
        r.sleep()
        
    arduino_data.close()
