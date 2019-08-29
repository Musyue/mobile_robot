#! /usr/bin/env python
# coding=utf-8
import rospy
import sys
from std_msgs.msg import String,Float64,Bool,Int64MultiArray
# sys.path.append("..")
from ctypes import *
import yaml
import os
from math import *
from bitstring import Bits

from config.command import *
from can_analysis.can_analysis_driver import *
import time
from mobile_control.mobileplatform_driver import *
import binascii
from mobile_robot.msg import irr_encode_msg
import numpy 

from geometry_msgs.msg import Twist
class MobilePlatFormKeyboardControl():
    def __init__(self,):
        self.wheel_R=0.15/2#m
        self.car_length=0.5
        self.car_width=0.395
        # self.MobileControl=MobilePlatformDriver()#init can analysis
        self.Abs_Encoder_fl_id1_oct_buffer=[]
        self.Abs_Encoder_fr_id2_oct_buffer=[]
        self.Abs_Encoder_rl_id3_oct_buffer=[]
        self.Abs_Encoder_rr_id4_oct_buffer=[]

        ###walking
        self.Driver_walk_encode_fl_buffer=[]
        self.Driver_walk_encode_fr_buffer=[]
        self.Driver_walk_encode_rl_buffer=[]
        self.Driver_walk_encode_rr_buffer=[] 
        #walking velocity
        self.Driver_walk_velocity_encode_fl_buffer=[]
        self.Driver_walk_velocity_encode_fr_buffer=[]
        self.Driver_walk_velocity_encode_rl_buffer=[]
        self.Driver_walk_velocity_encode_rr_buffer=[]     
        ###steer
        self.Driver_steer_encode_fl_buffer=[]
        self.Driver_steer_encode_fr_buffer=[]
        self.Driver_steer_encode_rl_buffer=[]
        self.Driver_steer_encode_rr_buffer=[]
        ####ROS

        self.linear_x=0
        self.linear_y=0
        self.linear_z=0
        self.angular_x=0
        self.angular_y=0
        self.angular_z=0

        self.driver_sensor_feedback_sub = rospy.Subscriber("/mobile_platform_driver_sensor_feedback", Int64MultiArray, self.sub_platform_sensor_callback)
        self.cmd_vel_data_sub = rospy.Subscriber("/cmd_vel", Twist, self.sub_cmd_vel_callback)
    def sub_platform_sensor_callback(self,msg):
        self.dynamic_array(self.Abs_Encoder_fl_id1_oct_buffer,msg.Abs_Encoder_fl_id1_oct)
        self.dynamic_array(self.Abs_Encoder_fr_id2_oct_buffer,msg.Abs_Encoder_fr_id2_oct)
        self.dynamic_array(self.Abs_Encoder_rl_id3_oct_buffer,msg.Abs_Encoder_rl_id3_oct)
        self.dynamic_array(self.Abs_Encoder_rr_id4_oct_buffer,msg.Abs_Encoder_rr_id4_oct)

        self.dynamic_array(self.Driver_walk_encode_fl_buffer,msg.Driver_walk_encode_fl)
        self.dynamic_array(self.Driver_walk_encode_fr_buffer,msg.Driver_walk_encode_fr)
        self.dynamic_array(self.Driver_walk_encode_rl_buffer,msg.Driver_walk_encode_rl)
        self.dynamic_array(self.Driver_walk_encode_rr_buffer,msg.Driver_walk_encode_rr) 

        self.dynamic_array(self.Driver_walk_velocity_encode_fl_buffer,msg.Driver_walk_velocity_encode_fl)
        self.dynamic_array(self.Driver_walk_velocity_encode_fr_buffer,msg.Driver_walk_velocity_encode_fr)
        self.dynamic_array(self.Driver_walk_velocity_encode_rl_buffer,msg.Driver_walk_velocity_encode_rl)
        self.dynamic_array(self.Driver_walk_velocity_encode_rr_buffer,msg.Driver_walk_velocity_encode_rr)     

        self.dynamic_array(self.Driver_steer_encode_fl_buffer,msg.Driver_steer_encode_fl)
        self.dynamic_array(self.Driver_steer_encode_fr_buffer,msg.Driver_steer_encode_fr)
        self.dynamic_array(self.Driver_steer_encode_rl_buffer,msg.Driver_steer_encode_rl)
        self.dynamic_array(self.Driver_steer_encode_rr_buffer,msg.Driver_steer_encode_rr)
    def sub_cmd_vel_callback(self,msg):
        self.linear_x=msg.linear.x
        self.linear_y=msg.linear.y
        self.linear_z=msg.linear.z
        self.angular_x=msg.angular.x
        self.angular_y=msg.angular.y
        self.angular_z=msg.angular.z
    def dynamic_array(self,listdata,newdata):
        if len(listdata)>=10:
            listdata=listdata[1:]
            listdata.append(newdata)
        else:
            listdata.append(newdata)

    def Init_Ros_Node(self):
        rospy.init_node("cmd_vel_control_for_mobileplatform")

    def caculate_velocity(self,vel):
        return (60*vel)/(0.15*pi)
    def caculate_steer_degree_thetafr_re(self):
        """
        arccot(x)=
        {
            arctan1/x+π(x>0)
            arctan1/x(x<0)
        }
        """
        print self.linear_x,self.angular_z
        if self.linear_x!=0:
            thetafr=atan(self.angular_z*self.car_length/(2*self.linear_x))
            thetare=atan(-tan(thetafr))
            temp_fo=
            return [thetafr,thetafr]
    def degree_to_pulse(self,degree):
        #degree 弧度
        return (degree*(220*1024*4)/pi)
    def output_pulse_position_control(self,left_right,degree_tar):
        temp_left=['left','LEFT']
        temp_right=['right','RIGHT']

        if 0 not in [len(self.Driver_steer_encode_fl_buffer),len(self.Driver_steer_encode_fr_buffer),len(self.Driver_steer_encode_rl_buffer),len(self.Driver_steer_encode_rr_buffer)]:
            oldpusle_fl=self.Driver_steer_encode_fl_buffer[-1]
            oldpusle_fr=self.Driver_steer_encode_fr_buffer[-1]
            oldpusle_rl=self.Driver_steer_encode_rl_buffer[-1]
            oldpusle_rr=self.Driver_steer_encode_rr_buffer[-1]
            #left_right 轮子向左转还是向右转，右转脉冲加，左转脉冲减
            if left_right in temp_left:
                return [oldpusle_fl-self.degree_to_pulse(degree_tar),oldpusle_fr-self.degree_to_pulse(degree_tar),oldpusle_rl-self.degree_to_pulse(degree_tar),oldpusle_rr-self.degree_to_pulse(degree_tar)]
            elif left_right in temp_right:
                return [oldpusle_fl+self.degree_to_pulse(degree_tar),oldpusle_fr+self.degree_to_pulse(degree_tar),oldpusle_rl+self.degree_to_pulse(degree_tar),oldpusle_rr+self.degree_to_pulse(degree_tar)]
            else:
                pass
        else:
            print "there is no data from driver feedback---please check----"
    def Init_mobile_driver(self):
        self.MobileControl.Opreation_Controller_Mode(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.SET_MODE_POSITION)
        # time.sleep(0.1)
        self.MobileControl.Opreation_Controller_Mode(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.SET_MODE_POSITION)
        # time.sleep(0.1)
        self.MobileControl.Opreation_Controller_Mode(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'],self.MobileControl.MobileDriver_Command.SET_MODE_VELOCITY)
        # time.sleep(0.1)
        self.MobileControl.Opreation_Controller_Mode(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'],self.MobileControl.MobileDriver_Command.SET_MODE_VELOCITY)
        self.MobileControl.Enable_Steering_Controller()
        self.MobileControl.Enable_Walking_Controller()

def main():

    mpfh=MobilePlatFormKeyboardControl()
    mpfh.Init_Ros_Node()
    # mpfh.Init_mobile_driver()
    ratet=1
    rate = rospy.Rate(ratet)
    data_array=[]
    pubmsg=irr_encode_msg()


    while not rospy.is_shutdown():  
        # print "haha"
        print mpfh.caculate_steer_degree_thetafr_re()
        rate.sleep() 
    mpfh.MobileControl.CanAnalysis.Can_VCICloseDevice()
   
if __name__=="__main__":
    main()