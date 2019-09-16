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

import numpy as np
import sys, select, termios, tty
from geometry_msgs.msg import Twist
import readchar
class MobilePlatFormKeyboardControl():
    def __init__(self,):
        self.wheel_R=0.15/2#m
        self.MobileControl=MobilePlatformDriver()#init can analysis
        self.Abs_Encoder_fl_id1_oct=0
        self.Abs_Encoder_fr_id2_oct=0
        self.Abs_Encoder_rl_id3_oct=0
        self.Abs_Encoder_rr_id4_oct=0
        self.fl_abs_encode=self.MobileControl.CanAnalysis.yamlDic['Homing_abs_encoder_data']['fl']
        self.fr_abs_encode=self.MobileControl.CanAnalysis.yamlDic['Homing_abs_encoder_data']['fr']
        self.rl_abs_encode=self.MobileControl.CanAnalysis.yamlDic['Homing_abs_encoder_data']['rl']
        self.rr_abs_encode=self.MobileControl.CanAnalysis.yamlDic['Homing_abs_encoder_data']['rr']
        self.limit_error=self.MobileControl.CanAnalysis.yamlDic['Homing_error_limit']
        ###walking
        self.Driver_walk_encode_fl=0
        self.Driver_walk_encode_fr=0
        self.Driver_walk_encode_rl=0
        self.Driver_walk_encode_rr=0   
        #walking velocity
        self.Driver_walk_velocity_encode_fl=0
        self.Driver_walk_velocity_encode_fr=0
        self.Driver_walk_velocity_encode_rl=0
        self.Driver_walk_velocity_encode_rr=0      
        ###steer
        self.Driver_steer_encode_fl=0
        self.Driver_steer_encode_fr=0
        self.Driver_steer_encode_rl=0
        self.Driver_steer_encode_rr=0
        self.Driver_steer_encode_fl_original=0
        self.Driver_steer_encode_fr_original=0
        self.Driver_steer_encode_rl_original=0
        self.Driver_steer_encode_rr_original=0
        ####ROS


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
m: control robot wheel turn left
.: control robot hweel  turn right
b: go back initial point
y: set speed not zero
a: control robot wheel turn left circle
d: control robot wheel turn right circle
q/z : increase/decrease max speeds by 10%
r:    control steer's axis to the center of circle
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

    def List_to_HEXList(self,Listdata):
        temp=[]
        for i in Listdata:
            temp.append(hex(i))
        return temp
    def HEX_String_List_To_Oct(self,Hexstrlist):
        temp=[]
        for i in Hexstrlist:
            temp.append("0x{:02x}".format(i))
        # print temp
        hex03=temp[3]
        hex04=temp[4]
        newtemp=int(''.join([hex04,hex03]).replace('0x',''),16)#str([hex04,hex03])
        return newtemp
    def HEX_String_List_To_Oct_Four(self,Octlist):
        # print "Hexstrlist",Hexstrlist
        temp=[]
        for i in Octlist:
            temp.append(i)
        # print temp
        
        hex04=temp[4]
        hex05=temp[5]
        hex06=temp[6]
        hex07=temp[7]
        return -1*(~(temp[7]<<24|temp[6]<<16|temp[5]<<8|temp[4]-1)&0xFFFFFFFF)
        # newtemp=int(''.join([hex07,hex06,hex05,hex04]).replace('0b',''),2)#str([hex04,hex03])
        # kkk=''.join([hex07,hex06,hex05,hex04]).replace('0b','')
        # # print newtemp,Bits(bin=kkk).int
        # return Bits(bin=kkk).int
    def New_Read_Encoder_data_From_ABS_Encoder(self,RecNum):
        # transmit_status_1=self.MobileControl.CanAnalysis.Can_Transmit(0,1,1,8,self.MobileControl.MobileDriver_Command.REQUEST_ENCODER_1)
        # time.sleep(0.0015)
        # transmit_status_2=self.MobileControl.CanAnalysis.Can_Transmit(0,1,2,8,self.MobileControl.MobileDriver_Command.REQUEST_ENCODER_2)
        # time.sleep(0.0015)
        # transmit_status_3=self.MobileControl.CanAnalysis.Can_Transmit(0,1,3,8,self.MobileControl.MobileDriver_Command.REQUEST_ENCODER_3)
        # time.sleep(0.0015)
        # transmit_status_4=self.MobileControl.CanAnalysis.Can_Transmit(0,1,4,8,self.MobileControl.MobileDriver_Command.REQUEST_ENCODER_4)
        # time.sleep(0.0015)
        #position feedback
        position_data_fl=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_POSITION_FEEDBACK)
        # time.sleep(0.0015)
        position_data_fr=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_POSITION_FEEDBACK)
        # time.sleep(0.0015)
        position_data_rl=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_POSITION_FEEDBACK)
        # time.sleep(0.0015)
        position_data_rr=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_POSITION_FEEDBACK)
        # time.sleep(0.0015)

        # position_data_walk_fl=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_POSITION_FEEDBACK)
        # time.sleep(0.0015)
        # position_data_walk_fr=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_POSITION_FEEDBACK)
        # time.sleep(0.0015)
        # position_data_walk_rl=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_POSITION_FEEDBACK)
        # time.sleep(0.0015)
        # position_data_walk_rr=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_POSITION_FEEDBACK)
        # time.sleep(0.0015)
        position_data_walk_vel_fl=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_VELOCITY_FEEDBACK)
        time.sleep(0.0015)
        position_data_walk_vel_fr=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_VELOCITY_FEEDBACK)
        time.sleep(0.0015)
        position_data_walk_vel_rl=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_VELOCITY_FEEDBACK)
        time.sleep(0.0015)
        position_data_walk_vel_rr=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_VELOCITY_FEEDBACK)
        time.sleep(0.0015)
        ret,kk=self.MobileControl.CanAnalysis.Can_New_Receive(0,RecNum)
        time.sleep(0.03)
        if ret:
            for i in range(RecNum):
                # if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and list(kk[i].Data)[1]==1:
                #     # print('abs encoder 1',self.List_to_HEXList(list(kk[i].Data)))
                #     # print(kk[i].DataLen)
                #     # self.Abs_Encoder_fl_id1.append(self.List_to_HEXList(list(kk[i].Data)))
                #     self.Abs_Encoder_fl_id1_oct=self.HEX_String_List_To_Oct(list(kk[i].Data))
                #     self.MobileControl.logger.loggerinfo(self.HEX_String_List_To_Oct(list(kk[i].Data)))
                # if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and list(kk[i].Data)[1]==2:
                #     # print('abs encoder 2',self.List_to_HEXList(list(kk[i].Data)))
                #     # print(kk[i].DataLen)
                #     # self.Abs_Encoder_fr_id2.append(self.List_to_HEXList(list(kk[i].Data)))
                #     self.Abs_Encoder_fr_id2_oct=self.HEX_String_List_To_Oct(list(kk[i].Data))
                #     self.MobileControl.logger.loggerinfo(self.HEX_String_List_To_Oct(list(kk[i].Data)))
                # if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and list(kk[i].Data)[1]==3:
                #     # print('abs Encoder 3',self.List_to_HEXList(list(kk[i].Data)))
                #     # print(kk[i].DataLen)
                #     # self.Abs_Encoder_rl_id3.append(self.List_to_HEXList(list(kk[i].Data)))
                #     self.MobileControl.logger.loggerinfo(self.HEX_String_List_To_Oct(list(kk[i].Data)))
                #     self.Abs_Encoder_rl_id3_oct=self.HEX_String_List_To_Oct(list(kk[i].Data))
                # if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and list(kk[i].Data)[1]==4:
                #     # print('abs Encoder 4',self.List_to_HEXList(list(kk[i].Data)))
                #     # print(kk[i].DataLen)
                #     # self.Abs_Encoder_rr_id4.append(self.List_to_HEXList(list(kk[i].Data)))
                    
                #     self.Abs_Encoder_rr_id4_oct=self.HEX_String_List_To_Oct(list(kk[i].Data))

                #     self.MobileControl.logger.loggerinfo(self.HEX_String_List_To_Oct(list(kk[i].Data)))
                # print "kk[i].ID",kk[i].ID
                # print  "list(kk[i].Data)",list(kk[i].Data)
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x583 and list(kk[i].Data)[2]==96 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    # print('driver position encode fl',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_steer_encode_fl=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))
                    # print "self.Driver_steer_encode_fl",self.Driver_steer_encode_fl
                    # print "list(kk[i].Data)",list(kk[i].Data)
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x583 and list(kk[i].Data)[2]==104 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    # print('driver position encode fr',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_steer_encode_fr=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))
                    # print " self.Driver_steer_encode_fr", self.Driver_steer_encode_fr
                    # print "list(kk[i].Data)",list(kk[i].Data)
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x584 and list(kk[i].Data)[2]==96 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    # print('driver position encode rl',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_steer_encode_rl=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))
                    # print 'self.Driver_steer_encode_rl',self.Driver_steer_encode_rl
                    # print "list(kk[i].Data)",list(kk[i].Data)
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x584 and list(kk[i].Data)[2]==104 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    # print('driver position encode rr',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_steer_encode_rr=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))         
                    # print "self.Driver_steer_encode_rr",self.Driver_steer_encode_rr
                    # print "list(kk[i].Data)",list(kk[i].Data)
                ###walk
                # if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x581 and list(kk[i].Data)[1]==100 and list(kk[i].Data)[2]==96 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                #     # print('driver position walk encode fl',self.List_to_HEXList(list(kk[i].Data)))
                #     self.Driver_walk_encode_fl=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))
                #     # print "self.Driver_walk_encode_fl",self.Driver_walk_encode_fl
                #     # print "list(kk[i].Data)",list(kk[i].Data)
                # if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x581 and list(kk[i].Data)[1]==100 and list(kk[i].Data)[2]==104 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                #     # print('driver position encode fr',self.List_to_HEXList(list(kk[i].Data)))
                #     self.Driver_walk_encode_fr=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))
                #     # print " self.Driver_walk_encode_fr", self.Driver_walk_encode_fr
                #     # print "list(kk[i].Data)",list(kk[i].Data)
                # if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x582 and list(kk[i].Data)[1]==100 and list(kk[i].Data)[2]==96 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                #     # print('driver position encode rl',self.List_to_HEXList(list(kk[i].Data)))
                #     self.Driver_walk_encode_rl=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))
                #     # print 'self.Driver_walk_encode_rl',self.Driver_walk_encode_rl
                #     # print "list(kk[i].Data)",list(kk[i].Data)
                # if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x582 and list(kk[i].Data)[1]==100 and list(kk[i].Data)[2]==104 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                #     # print('driver position encode rr',self.List_to_HEXList(list(kk[i].Data)))
                #     self.Driver_walk_encode_rr=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))         
                #     # print "self.Driver_walk_encode_rr",self.Driver_walk_encode_rr
                #     # print "list(kk[i].Data)",list(kk[i].Data)
                # ###walk velocity

                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x581 and list(kk[i].Data)[1]==105 and list(kk[i].Data)[2]==96 :#and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    # print('driver velocity walk encode fl',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_walk_velocity_encode_fl=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))
                    # print "self.Driver_walk_velocity_encode_fl",self.Driver_walk_velocity_encode_fl
                    # print "list(kk[i].Data)",list(kk[i].Data)
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x581 and list(kk[i].Data)[1]==105 and list(kk[i].Data)[2]==104 :#and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    # print('driver velocity walk encode fr',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_walk_velocity_encode_fr=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))
                    # print " self.Driver_walk_velocity_encode_fr", self.Driver_walk_velocity_encode_fr
                    # print "list(kk[i].Data)",list(kk[i].Data)
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x582 and list(kk[i].Data)[1]==105 and list(kk[i].Data)[2]==96:# and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    # print('driver velocity walk encode rl',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_walk_velocity_encode_rl=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))
                    # print 'self.Driver_walk_velocity_encode_rl',self.Driver_walk_velocity_encode_rl
                    # print "list(kk[i].Data)",list(kk[i].Data)
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x582 and list(kk[i].Data)[1]==105 and list(kk[i].Data)[2]==104:# and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    # print('driver velocity walk encode rr',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_walk_velocity_encode_rr=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))         
                    # print "self.Driver_walk_velocity_encode_rr",self.Driver_walk_velocity_encode_rr
                    # print "list(kk[i].Data)",list(kk[i].Data)

    def dynamic_array(self,listdata,newdata):
        if len(listdata)>=10:
            listdata=listdata[1:]
            listdata.append(newdata)
        else:
            listdata.append(newdata)

    def Init_Ros_Node(self):
        rospy.init_node("keyboard_control_for_mobileplatform")

    def vels(self,speed,turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)
    def caculate_velocity(self,vel):
        return (60*vel)/(0.15*pi)
    def degree_to_pulse(self,rad):
        #degree 弧度
        return (rad*(220*1024*4.0)/(2.0*pi))
    def output_pulse_position_control_zero(self,flag_list,degree_tar):
        """
        flag_list=[]
        """
        oldpusle_fl=self.Driver_steer_encode_fl_original
        oldpusle_fr=self.Driver_steer_encode_fr_original
        oldpusle_rl=self.Driver_steer_encode_rl_original
        oldpusle_rr=self.Driver_steer_encode_rr_original
        return [oldpusle_fl+flag_list[0]*self.degree_to_pulse(degree_tar),oldpusle_fr+flag_list[1]*self.degree_to_pulse(degree_tar),oldpusle_rl+flag_list[2]*self.degree_to_pulse(degree_tar),oldpusle_rr+flag_list[3]*self.degree_to_pulse(degree_tar)]

    def output_pulse_position_control(self,flag_list,degree_tar):
        """
        flag_list=[]
        """
        oldpusle_fl=self.Driver_steer_encode_fl_original
        oldpusle_fr=self.Driver_steer_encode_fr_original
        oldpusle_rl=self.Driver_steer_encode_rl_original
        oldpusle_rr=self.Driver_steer_encode_rr_original
        # oldpusle_fl=self.Driver_steer_encode_fl
        # oldpusle_fr=self.Driver_steer_encode_fr
        # oldpusle_rl=self.Driver_steer_encode_rl
        # oldpusle_rr=self.Driver_steer_encode_rr
        return [oldpusle_fl+flag_list[0]*self.degree_to_pulse(degree_tar),oldpusle_fr+flag_list[1]*self.degree_to_pulse(degree_tar),oldpusle_rl+flag_list[2]*self.degree_to_pulse(degree_tar),oldpusle_rr+flag_list[3]*self.degree_to_pulse(degree_tar)]

        # else:
        #     print "there is no data from driver feedback---please check----"
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
    
    def set_original_sensor_data(self,original_data_list):
        self.Driver_steer_encode_fl_original=original_data_list[0]
        self.Driver_steer_encode_fr_original=original_data_list[1]
        self.Driver_steer_encode_rl_original=original_data_list[2]
        self.Driver_steer_encode_rr_original=original_data_list[3]
def getKey():
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
def main():
    # settings = termios.tcgetattr(sys.stdin)

    mpfh=MobilePlatFormKeyboardControl()


    mpfh.Init_Ros_Node()


    # mpfh.MobileControl.CanAnalysis.Can_ReadBoardInfo()
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
        'd':(0,0),
        'r':(0,0),
        'b':(0,0)
           }
    speed = .1
    turn = .1
    flg=1
    try:
        print mpfh.strmessage
        print mpfh.vels(speed,turn)
        flag=1
        while not rospy.is_shutdown():
            if 0 not in [mpfh.Driver_steer_encode_fl_original,mpfh.Driver_steer_encode_fr_original,mpfh.Driver_steer_encode_rl_original,mpfh.Driver_steer_encode_rr_original]:
                if flg:
                    mpfh.Init_mobile_driver()
                    OutputPulse=mpfh.output_pulse_position_control_zero([.0,.0,.0,.0],turn)
                    print "PositionData:",OutputPulse
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[0]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[1]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[2]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[3]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                    flg=0
                recevenum=mpfh.MobileControl.CanAnalysis.Can_GetReceiveNum(0)
                if recevenum!=None:
                    data_array=[
                    mpfh.Abs_Encoder_fl_id1_oct,mpfh.Abs_Encoder_fr_id2_oct,mpfh.Abs_Encoder_rl_id3_oct,mpfh.Abs_Encoder_rr_id4_oct,mpfh.Driver_walk_encode_fl,
                    mpfh.Driver_walk_encode_fr,mpfh.Driver_walk_encode_rl,mpfh.Driver_walk_encode_rr,mpfh.Driver_steer_encode_fl,mpfh.Driver_steer_encode_fr,
                    mpfh.Driver_steer_encode_rl,mpfh.Driver_steer_encode_rr,mpfh.Driver_walk_velocity_encode_fl,mpfh.Driver_walk_velocity_encode_fr,
                    mpfh.Driver_walk_velocity_encode_rl,mpfh.Driver_walk_velocity_encode_rr
                    
                    ]
                    mpfh.New_Read_Encoder_data_From_ABS_Encoder(recevenum)

                    print "data_array",data_array
                    if 0 not in [mpfh.Driver_steer_encode_fl,mpfh.Driver_steer_encode_fr,mpfh.Driver_steer_encode_rl,mpfh.Driver_steer_encode_rr]:
                        key = readchar.readkey()
                        if flag:
                            mpfh.set_original_sensor_data([mpfh.Driver_steer_encode_fl,mpfh.Driver_steer_encode_fr,mpfh.Driver_steer_encode_rl,mpfh.Driver_steer_encode_rr])
                            flag=0
                        if key==None:
                            continue
                        print "key----",key
                        # key=0
                        # 运动控制方向键（1：正方向，-1负方向）
                        if key in moveBindings.keys():
                            if key=='a':
                                VelocityData= mpfh.caculate_velocity(-1.0*speed)
                                print "VelocityData:RPM/Min",VelocityData
                                mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),'left',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'])
                                mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),'right',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'])
                                mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),'left',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'])
                                mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),'right',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'])
                            elif key =='d':
                                VelocityData= mpfh.caculate_velocity(speed)
                                print "VelocityData:RPM/Min",VelocityData
                                mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),'left',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'])
                                mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),'right',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'])
                                mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),'left',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'])
                                mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),'right',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'])
                            
                            elif key=='i':
                                VelocityData= mpfh.caculate_velocity(-1.0*speed)
                                print "VelocityData:RPM/Min",VelocityData
                                mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),'left',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'])
                                mpfh.MobileControl.Send_Velocity_Driver(int(VelocityData),'right',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'])
                                mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),'left',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'])
                                mpfh.MobileControl.Send_Velocity_Driver(int(VelocityData),'right',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'])
                            elif key ==',':
                                VelocityData= mpfh.caculate_velocity(speed)
                                print "VelocityData:RPM/Min",VelocityData
                                mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),'left',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'])
                                mpfh.MobileControl.Send_Velocity_Driver(int(VelocityData),'right',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'])
                                mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),'left',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'])
                                mpfh.MobileControl.Send_Velocity_Driver(int(VelocityData),'right',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'])
                            elif key =='j':
                                OutputPulse=mpfh.output_pulse_position_control([-1.0,-1.0,-1.0,-1.0],pi/2)
                                print "PositionData:",OutputPulse
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[0]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[1]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[2]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[3]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                            elif key =='l':
                                OutputPulse=mpfh.output_pulse_position_control([1.0,1.0,1.0,1.0],pi/2)
                                print "PositionData:",OutputPulse
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[0]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[1]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[2]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[3]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                            elif key == 'u':
                                OutputPulse=mpfh.output_pulse_position_control([-1.0,-1.0,-1.0,-1.0],pi/4)
                                print "PositionData:",OutputPulse
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[0]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[1]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[2]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[3]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                            elif key == 'o':
                                OutputPulse=mpfh.output_pulse_position_control([1.0,1.0,1.0,1.0],pi/4)
                                print "PositionData:",OutputPulse
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[0]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[1]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[2]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[3]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                            elif key == 'm':
                                OutputPulse=mpfh.output_pulse_position_control([-1.0,1.0,1.0,-1.0],turn)
                                print "PositionData",OutputPulse
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[0]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[1]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[2]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[3]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                            elif key == 'r':
                                OutputPulse=mpfh.output_pulse_position_control([1.0,-1.0,-1.0,1.0],pi/4)
                                print "PositionData",OutputPulse
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[0]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[1]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[2]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[3]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])                    
                            elif key == '.':
                                OutputPulse=mpfh.output_pulse_position_control([-1.0,1.0,1.0,-1.0],turn)
                                print "PositionData:",OutputPulse
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[0]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[1]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[2]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[3]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                            elif key == 'b':
                                OutputPulse=mpfh.output_pulse_position_control_zero([.0,.0,.0,.0],turn)
                                print "PositionData:",OutputPulse
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[0]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[1]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[2]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[3]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                            else:
                                pass
                            count = 0
                        # 速度修改键
                        elif key in speedBindings.keys():
                            speed = speed * speedBindings[key][0]  # 线速度增加0.1倍
                            turn = turn * speedBindings[key][1]    # 角速度增加0.1倍
                            count = 0
                            if speed>=3:
                                speed=0.1
                            print mpfh.vels(speed,turn)
                            if (status == 14):
                                print mpfh.strmessage
                            status = (status + 1) % 15
                        # 停止键
                        elif key == ' ' or key =='k':
                            mpfh.MobileControl.Send_Velocity_Driver(-1*int(0),'left',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'])
                            mpfh.MobileControl.Send_Velocity_Driver(-1*int(0),'right',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'])
                            mpfh.MobileControl.Send_Velocity_Driver(-1*int(0),'left',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'])
                            mpfh.MobileControl.Send_Velocity_Driver(-1*int(0),'right',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'])
                            
                            # speed = 0
                            # turn = 0
                        elif key == 'y' :
                            speed = 0.2
                            turn = 0.1
                        else:
                            count = count + 1
                            if (key == '\x03'):
                                break
                    else:
                        print "wait data readkeyfrom driver-----"
                else:
                    mpfh.New_Read_Encoder_data_From_ABS_Encoder(recevenum)
                rate.sleep()
            else:
                print "wait data from driver data from feedback topic ----------"
                time.sleep(0.5)
    except:
        print e

    finally:
        mpfh.MobileControl.Save_Parameter(1)
        time.sleep(3)
        mpfh.MobileControl.Save_Parameter(1)
        # time.sleep(3)
        print "key board control over-------"
        # twist = Twist()
        # twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        # pub.publish(twist)


    mpfh.MobileControl.CanAnalysis.Can_VCICloseDevice()
   
if __name__=="__main__":
    main()