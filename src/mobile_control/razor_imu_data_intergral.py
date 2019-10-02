#! /usr/bin/env python
# coding=utf-8
import rospy
import sys
from std_msgs.msg import String,Float64,Bool,Int64MultiArray
from sensor_msgs.msg import Imu
import time 
from math import *
# from can_analysis.can_analysis_driver import *
from mobile_control.mobileplatform_driver_steptech import *
# from mobile_control.mobileplatform_cmd_vel_ros_steptech import MobilePlatFormKeyboardControl
class IMUDATAINTERGAL():
    def __init__(self):
        self.mpfh=MobilePlatformDriver()
        self.wheel_R=0.15/2#m
        self.car_length=0.5
        self.car_width=0.395
        self.imu_sub=rospy.Subscriber('/imu_data',Imu,self.Imu_callback)
        self.ImuOrientation=()
        self.ImuAngularvelocity=()
        self.ImuLinearAcceleration=()
        self.ImuOrientationCovariance=[]
        self.ImuAngularvelocityCovariance=[]
        self.ImuLinearAccelerationCovariance=[]
        
    def Init_Ros_Node(self):
        
        rospy.init_node("imu_data_for_mobileplatform")
    def Imu_callback(self,msg):
        self.ImuOrientation=(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
        self.ImuAngularvelocity=(msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z)
        self.ImuLinearAcceleration=(msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z)
        self.ImuOrientationCovariance=msg.orientation_covariance
        self.ImuAngularvelocityCovariance=msg.angular_velocity_covariance
        self.ImuLinearAccelerationCovariance=msg.linear_acceleration_covariance
def main():
   imuobj=IMUDATAINTERGAL()
   imuobj.Init_Ros_Node()
   ratet=10
   rate=rospy.Rate(ratet)
   zerotime=time.time()
   theta=0#(x,y,theta)
   gama=0#(roation angular)
   v=0.1331
   dt=0
   x=0
   y=0
   while not rospy.is_shutdown():
        recevenum=imuobj.mpfh.CanAnalysis.Can_GetReceiveNum(0)
        print "recevenum",recevenum
        if recevenum!=None:
            imuobj.mpfh.Read_sensor_data_from_driver()
            starttime=time.time()
            # print imuobj.ImuOrientation
            print "starttime",starttime
            thetastar=v*tan(gama)/imuobj.car_length
            print "thetastar",thetastar
            theta+=thetastar*dt
            xstar=v*cos(theta)
            ystar=v*sin(theta)
            x+=xstar*dt
            y+=ystar*dt
            print "x,y,theata",x,y,theta
            endtime=time.time() 
            dt=endtime-starttime
            print imuobj.mpfh.Driver_walk_velocity_encode_fl
        else:
            imuobj.mpfh.Send_Control_Command( imuobj.mpfh.CanAnalysis.yamlDic['sync_data_ID'], imuobj.mpfh.MobileDriver_Command.ZERO_COMMAND)
        
        rate.sleep() 
if __name__=="__main__":
    main()