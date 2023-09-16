from tracemalloc import start
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import os
import socket
import string
import numpy as np
import cv2
import serial
#import keyboard
import sys
import select
import tty
import termios

#run mavproxy
#sudo chmod 666 /dev/ttyTHS1
#mavproxy.py --master=/dev/ttyUSB0 --baudrate=115200 --out=udp:0.0.0.0:14550 --out=udp:iplaptop:14551

#dronekit
vehicle = None

# Arduino connection 
connection_bus='/dev/ttyUSB0'
baud=57600
wait_ready_ardu=0
global_counter = 0


#PID lidar wall following
lidar_depan = 0; lidar_bawah = 0; lidar_kanan = 0; lidar_kiri = 0;

#PID Yolo
KP_yolo = 0.0002
KD_yolo = 0.0002
KI_yolo = 0.0001
MAXSPEEDXY = 0.35


#Speed and Altitude

KETINGGIAN_MISI_INDOOR = 100 #cm
SPEED_CHANGE_ALTITUDE = 0.1
SPEED_XY = 0.18
change_alt_state = False

VX_ISINVERTED = True
VY_ISINVERTED = False

#global variable
posStrat = 0
step_mission = 0
message  = ''
vy = 0; vx = 0; vz= 0
vxRotated = 0; vyRotated = 0;
countersend = 0
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

def velocity (x, y, z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            x, y, z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)
    vehicle.send_mavlink(msg)


def change_direction(target, real, speed, threshold):
    if real<(target-threshold):
        output=speed
    if real>(target+threshold):
        output=-speed
    if real<=(target+threshold) and real>=(target-threshold):
        output=0.0
    return output


def IK_YAW_COMPASS (velocityx, velocityy):
    new_x=round (((velocityx*(math.cos(vehicle.attitude.yaw))) - (velocityy*(math.sin(vehicle.attitude.yaw)))),1)
    new_y=round (((velocityy*(math.sin(vehicle.attitude.yaw))) + (velocityx*(math.cos(vehicle.attitude.yaw)))),1)
    return new_x, new_y

def isData():
	return select.select([sys.stdin],[],[],0)==([sys.stdin],[],[])

def user_input():
    global posStrat
    if isData():
        val = sys.stdin.read(1)
        if (val == 'a' or val == 'A'):
            posStrat = 1
        if (val == 's' or val == 'S'):
            posStrat = 2


if __name__ == '__main__':

    old_settings = termios.tcgetattr(sys.stdin)
    arduino_data = serial.Serial(connection_bus,baud, timeout=1)
    #vehicle = connect('127.0.0.1:14550', wait_ready=True)
    vehicle = connect('/dev/ttyTHS1', wait_ready=True, baud=921600)
    #vehicle.wait_ready(True, raise_exception=False)
    try :
        tty.setcbreak(sys.stdin.fileno())
        while(1):
            data = arduino_data.readline()
            wait_ready_ardu+=1
            if wait_ready_ardu>15:
                lidar_depan, lidar_bawah, lidar_kanan, lidar_kiri = arduino_read(data)
                #posStrat = user_input()
                if posStrat> 0 :
                    if step_mission == 0:
                        vehicle.mode = VehicleMode("GUIDED")
                        global_counter+=1
                        if global_counter>=5 and lidar_bawah != 0: #delay sejenak utk arming
                            step_mission=2
                            global_counter=0

                    if step_mission == 2:
                        if vehicle.armed:
                            message='warming take off'
                            alt_warming=2
                            vehicle.simple_takeoff(alt_warming)
                            if lidar_bawah>=80:
                                global_counter = 0
                                change_alt_state=False
                                step_mission=3
                        else:
                            vehicle.armed=True
                    
                    
                    

                else:
                    arduino_data.write(b't')
                    message = 'Pres A strat A, pres S strat B'
                    user_input()

            new_x=round (((vx*(math.cos(vehicle.attitude.yaw))) - (vy*(math.sin(vehicle.attitude.yaw)))),1)
            new_y=round (((vy*(math.sin(vehicle.attitude.yaw))) + (vx*(math.cos(vehicle.attitude.yaw)))),1)
            #velocity(new_x,new_y, vz)
            
            print("[INFO] SYS INFO    = ", message)
            print("[INFO] LIDAR DEPAN = ", lidar_depan)
            print("[INFO] LIDAR BAWAH = ", lidar_bawah)
            print("[INFO] LIDAR KANAN = ", lidar_kanan)
            print("[INFO] LIDAR KIRI  = ", lidar_kiri)
            print("[INFO] VX NON IK   = ", new_x)
            print("[INFO] VY NON IK   = ", new_y)
            print("[INFO] VZ NON IK   = ", vz)
            print("[INFO] VX OUT IK   = ", vxRotated)
            print("[INFO] VY OUT IK   = ", vyRotated)
            print("[INFO] MISSIONSTEP = ", step_mission)
            print("[INFO] YAW         = ", math.degrees (vehicle.attitude.yaw))
            print("[INFO] POS STRAT   = ", posStrat)


                    
            #print info nilai
    

    except KeyboardInterrupt:
        print ("Killed by user")
    finally:
        print ("terminated")
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,old_settings) 
        arduino_data.close()
        vehicle.close()

