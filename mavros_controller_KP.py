#!/usr/bin/env python

##
#
# Control a MAV via mavros
#
##

import rospy
import tf
import math
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from std_msgs.msg    import Float64
from std_msgs.msg import Int32MultiArray



class MavController:
    """
    A simple object to help interface with mavros
    """
    def __init__(self):

        rospy.init_node("mav_control_node")
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_callback)
        rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.compass_callback)
        rospy.Subscriber("/arduinoLidar_pub", Int32MultiArray, self.lidar_array_callback)

        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)

        # mode 0 = STABILIZE
        # mode 4 = GUIDED
        # mode 9 = LAND
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        self.rc = RCIn()
        self.compass = Float64()
        self.lidarArray = Int32MultiArray()
        self.pose = Pose()
        self.timestamp = rospy.Time()

    def lidar_array_callback(self, data):
        self.lidarArray = data
    
    def compass_callback(self, data):
        self.compass = data

    def rc_callback(self, data):
        self.rc = data

    def pose_callback(self, data):
        """
        Handle local position information
        """
        self.timestamp = data.header.stamp
        self.pose = data.pose
   
    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
        """
        Send comand velocities. Must be in GUIDED mode. Assumes angular
        velocities are zero by default.
        """
        cmd_vel = Twist()

        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz

        cmd_vel.angular.x = avx
        cmd_vel.angular.y = avy
        cmd_vel.angular.z = avz

        self.cmd_vel_pub.publish(cmd_vel)

    def arm(self):
        """
        Arm the throttle
        """
        return self.arm_service(True)

    def disarm(self):
        """
        Disarm the throttle
        """
        return self.arm_service(False)

    def takeoff(self, height=1.0):
        """
        Arm the throttle, takeoff to a few feet, and set to guided mode
        """
        # Set to stabilize mode for arming
        #mode_resp = self.mode_service(custom_mode="0")
        mode_resp = self.mode_service(custom_mode="4")
        self.arm()

        # Set to guided mode
        #mode_resp = self.mode_service(custom_mode="4")

        # Takeoff
        takeoff_resp = self.takeoff_service(altitude=height)

        #return takeoff_resp
        return mode_resp

    def land(self):
        """
        Set in LAND mode, which should cause the UAV to descend directly,
        land, and disarm.
        """
        resp = self.mode_service(custom_mode="9")
        self.disarm()



if __name__=="__main__":
    ros_service = MavController()
    rospy.sleep(1)
    ros_service.takeoff(1) #takeoff 1 meter
    rospy.sleep(5)


    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #di isi sendiri vz tidak usah 
        #lalu di amati pada komponen vx dan vy mana yang kekanan kiri, depan belakang (BUKAN ARAH MATA ANGIN)
        vx = 0
        vy = 0.8
        vz = 0
        #===================================================

        lidar_Depan =  ros_service.lidarArray.data[0] 
        lidar_bawah =  ros_service.lidarArray.data[1]
        lidar_kanan =  ros_service.lidarArray.data[2]
        lidar_kiri =  ros_service.lidarArray.data[3] 
       
        
        compasRadians = math.radians(ros_service.compass.data)
        
        IKvx=round (( vx*math.cos(compasRadians) - vy*math.sin(compasRadians) ),2)
        IKvy=round (( vy*math.cos(compasRadians) + vx*math.sin(compasRadians) ),2)

        print ("Lidar  depan = ", lidar_Depan)
        print ("Lidar  bawah = ", lidar_bawah)
        print ("Lidar  kanan = ", lidar_kanan)
        print ("Lidar  kiri  = ", lidar_kiri)
        print ("Compass data = ", ros_service.compass.data)
        print ("Velocity X data =", IKvx)
        print ("Velocity Y data =", IKvy)

        ros_service.set_vel(IKvx, IKvy, vz)
        rate.sleep()
   
