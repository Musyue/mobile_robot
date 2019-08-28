#! /usr/bin/env python
# coding=utf-8
import rospy
import sys
from std_msgs.msg import String,Float64,Bool,Int64MultiArray
# sys.path.append("..")
from ctypes import *
import yaml
import os
from math import pi
from bitstring import Bits

from config.command import *
from can_analysis.can_analysis_driver import *
import time
from mobile_control.mobileplatform_driver import *
import binascii
from mobile_robot.msg import irr_encode_msg
import numpy as np
import sys, select, termios, tty
from geometry_msgs.msg import Twist
class MobilePlatFormKeyboardControl():
    def __init__(self,):
        self.wheel_R=0.15/2#m
        self.MobileControl=MobilePlatformDriver()#init can analysis
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


        self.driver_sensor_feedback_sub = rospy.Subscriber("/mobile_platform_driver_sensor_feedback", Int64MultiArray, self.sub_platform_sensor_callback)
        self.strmessage= """
Control irr robot!
---------------------------
Moving around:
u    i    o
j    k    l
m    ,    .
i: control robot forward 
,: control robot go back
j: control robot go left
l: control robot go right
u: control robot northwest
o: control robot eastwest
m: control robot turn left
.: control robot turn right
q/z : increase/decrease max speeds by 10%

w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""
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
    def dynamic_array(self,listdata,newdata):
        if len(listdata)>=10:
            listdata=listdata[1:]
            listdata.append(newdata)
        else:
            listdata.append(newdata)

    def Init_Ros_Node(self):
        rospy.init_node("keyboard_control_for_mobileplatform")
    def getKey(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    def vels(self,speed,turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)
    def caculate_velocity(self,vel):
        return (60*vel)/(0.15*pi)
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
    settings = termios.tcgetattr(sys.stdin)

    mpfh=MobilePlatFormKeyboardControl()


    mpfh.Init_Ros_Node()
    mpfh.Init_mobile_driver()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0

    ratet=1000

    rate = rospy.Rate(ratet)
    data_array=[]
    pubmsg=irr_encode_msg()
    
    speedBindings={
        'q':(1.1,1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }
    moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
        'a':(0,0),
        'd':(0,0)
           }
    speed = .1
    turn = .1
    try:
        print mpfh.strmessage
        print mpfh.vels(speed,turn)
        while not rospy.is_shutdown():  
            key = mpfh.getKey()
            # key=0
            # 运动控制方向键（1：正方向，-1负方向）
            if key in moveBindings.keys():
                if key=='i':
                    VelocityData= mpfh.caculate_velocity(speed)
                    print "VelocityData:RPM/Min",VelocityData
                    mpfh.MobileControl.Send_Velocity_Driver(int(VelocityData),'left',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'])
                    mpfh.MobileControl.Send_Velocity_Driver(int(VelocityData),'right',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'])
                    mpfh.MobileControl.Send_Velocity_Driver(int(VelocityData),'right',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'])
                    mpfh.MobileControl.Send_Velocity_Driver(int(VelocityData),'right',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'])
                elif key ==',':
                    VelocityData= -1*mpfh.caculate_velocity(speed)
                    print "VelocityData:RPM/Min",VelocityData
                    mpfh.MobileControl.Send_Velocity_Driver(int(VelocityData),'left',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'])
                    mpfh.MobileControl.Send_Velocity_Driver(int(VelocityData),'right',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'])
                    mpfh.MobileControl.Send_Velocity_Driver(int(VelocityData),'right',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'])
                    mpfh.MobileControl.Send_Velocity_Driver(int(VelocityData),'right',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'])
                elif key =='j':
                    OutputPulse=mpfh.output_pulse_position_control('left',pi/2)
                    print "PositionData:",OutputPulse
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                elif key =='l':
                    OutputPulse=mpfh.output_pulse_position_control('right',pi/2)
                    print "PositionData:",OutputPulse
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                elif key == 'u':
                    OutputPulse=mpfh.output_pulse_position_control('left',pi/4)
                    print "PositionData:",OutputPulse
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                elif key == 'o':
                    OutputPulse=mpfh.output_pulse_position_control('right',pi/4)
                    print "PositionData:",OutputPulse
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                elif key == 'm':
                    OutputPulse=mpfh.output_pulse_position_control('left',turn)
                    print "PositionData",OutputPulse
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                elif key == '.':
                    OutputPulse=mpfh.output_pulse_position_control('left',turn)
                    print "PositionData:",OutputPulse
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                else:
                    pass
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            # 速度修改键
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]  # 线速度增加0.1倍
                turn = turn * speedBindings[key][1]    # 角速度增加0.1倍
                count = 0

                print mpfh.vels(speed,turn)
                if (status == 14):
                    print mpfh.strmessage
                status = (status + 1) % 15
            # 停止键
            elif key == ' ' or key == 'k' :
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            # 目标速度=速度值*方向值
            target_speed = speed * x
            target_turn = turn * th

            # 速度限位，防止速度增减过快
            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            # 创建并发布twist消息
            twist = Twist()
            twist.linear.x = control_speed; 
            twist.linear.y = 0; 
            twist.linear.z = 0
            twist.angular.x = 0; 
            twist.angular.y = 0; 
            twist.angular.z = control_turn
            pub.publish(twist)

            rate.sleep() 

    except:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    mpfh.MobileControl.CanAnalysis.Can_VCICloseDevice()
   
if __name__=="__main__":
    main()