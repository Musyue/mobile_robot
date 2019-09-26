#! /usr/bin/env python
# coding=utf-8
import rospy
import sys
from std_msgs.msg import String,Float64,Bool,Int64MultiArray
from sensor_msgs.msg import Imu
class IMUDATAINTERGAL():
    def __init__(self,):
        self.wheel_R=0.15/2#m
        self.car_length=0.5
        self.car_width=0.395
        self.imu_sub=rospy.Subscriber('/imu_data',Imu,self.Imu_callback)
    def Init_Ros_Node(self):
        rospy.init_node("imu_data_for_mobileplatform")
    def Imu_callback(self,msg):
        print msg    
def main():
   imuobj=IMUDATAINTERGAL()
   imuobj.Init_Ros_Node()
   ratet=10
   rate=rospy.Rate(ratet)
   while not rospy.is_shutdown():
       print "1"
       rate.sleep() 
if __name__=="__main__":
    main()