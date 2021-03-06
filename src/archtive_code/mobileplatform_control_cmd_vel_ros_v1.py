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
import numpy as np
import sys, select, termios, tty
from geometry_msgs.msg import Twist
import readchar
class MobilePlatFormCmdVelControl():
    def __init__(self,):
        self.wheel_R=0.15/2#m
        self.car_length=0.5
        self.car_width=0.395
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

        self.linear_x=0
        self.linear_y=0
        self.linear_z=0
        self.angular_x=0
        self.angular_y=0
        self.angular_z=0
        self.cmd_vel_data_sub = rospy.Subscriber("/cmd_vel", Twist, self.sub_cmd_vel_callback)
        self.driver_position_feedback_sub = rospy.Subscriber("/mobile_platform_driver_position_feedback", Int64MultiArray, self.irr_data_callback)

    def irr_data_callback(self,msg):
        if 0 not in msg.data:
            # print msg.data
            self.Driver_steer_encode_fl_original=msg.data[0]
            self.Driver_steer_encode_fr_original=msg.data[1]
            self.Driver_steer_encode_rl_original=msg.data[2]
            self.Driver_steer_encode_rr_original=msg.data[3]
    def sub_cmd_vel_callback(self,msg):
        self.linear_x=msg.linear.x
        self.linear_y=msg.linear.y
        self.linear_z=msg.linear.z
        self.angular_x=msg.angular.x
        self.angular_y=msg.angular.y
        self.angular_z=msg.angular.z
    def caculate_bicycle_model_thetafr_re(self):

        # print self.linear_x,self.angular_z
        if self.linear_x!=0:
            thetafr=atan((self.angular_z*self.car_length)/(2.0*self.linear_x))
            thetare=atan(-tan(thetafr))
            return [thetafr,thetare]
        else:
            return [0.0,0.0]
    def my_arccot(self,x):
        return pi/2-atan(x)
        # if x>0:
        #     return atan(1/x)+pi
        # elif x<0:
        #     return atan(1/x)
        # else:
        #     return 0.0

    def caculate_four_steer_degree_theta(self):
        """
        arccot(x)=
        {
            arctan(1/x)+π(x>0)
            arctan(1/x)(x<0)
        }
        """
        temp_fr_re=self.caculate_bicycle_model_thetafr_re()
        # numpy.arccot()
        if 0 not in temp_fr_re:
            temp_theta_fo_fr=(1/tan(temp_fr_re[0]))*(1+(self.car_width/self.car_length)*(tan(temp_fr_re[0])-tan(temp_fr_re[1])))
            temp_theta_fi_fl=(1/tan(temp_fr_re[0]))*(1-(self.car_width/self.car_length)*(tan(temp_fr_re[0])-tan(temp_fr_re[1])))
            temp_theta_ro_rr=(1/tan(temp_fr_re[1]))*(1+(self.car_width/self.car_length)*(tan(temp_fr_re[0])-tan(temp_fr_re[1])))
            temp_theta_ri_rl=(1/tan(temp_fr_re[1]))*(1-(self.car_width/self.car_length)*(tan(temp_fr_re[0])-tan(temp_fr_re[1])))
            theta_fo_fr=self.my_arccot(temp_theta_fo_fr)
            theta_fi_fl=self.my_arccot(temp_theta_fi_fl)
            theta_ro_rr=self.my_arccot(temp_theta_ro_rr)
            theta_ri_rl=self.my_arccot(temp_theta_ri_rl)
            return [theta_fo_fr,temp_theta_fi_fl,temp_theta_ro_rr,temp_theta_ri_rl]
        else:
            # print "bicycle model temp_fr_re data may be not right------",temp_fr_re
            return [0.0,0.0,0.0,0.0]
    def caculate_four_walk_motor_velocity(self):
        temp_theta_fl_fr_rl_rr=self.caculate_four_steer_degree_theta()
        temp_fr_re=self.caculate_bicycle_model_thetafr_re()
        if 0.0 not in temp_fr_re:
            v1_fr_fo=(self.linear_x*tan(temp_fr_re[0]*(1/sin(temp_theta_fl_fr_rl_rr[0]))))/sqrt(1+(1/4)*(tan(temp_fr_re[0])-tan(temp_fr_re[1]))**2)
            v2_fl_fi=(self.linear_x*tan(temp_fr_re[0]*(1/sin(temp_theta_fl_fr_rl_rr[1]))))/sqrt(1+(1/4)*(tan(temp_fr_re[0])-tan(temp_fr_re[1]))**2)
            v3_rr_ro=(self.linear_x*tan(temp_fr_re[0]*(1/sin(temp_theta_fl_fr_rl_rr[2]))))/sqrt(1+(1/4)*(tan(temp_fr_re[0])-tan(temp_fr_re[1]))**2)
            v4_rl_ri=(self.linear_x*tan(temp_fr_re[0]*(1/sin(temp_theta_fl_fr_rl_rr[3]))))/sqrt(1+(1/4)*(tan(temp_fr_re[0])-tan(temp_fr_re[1]))**2)
            return [v1_fr_fo,v2_fl_fi,v3_rr_ro,v4_rl_ri] 
        else:
            # print "the mobile platform walking in line---"  
            return [self.linear_x,self.linear_x,self.linear_x,self.linear_x]             
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
        # time.sleep(0.0015)
        position_data_walk_vel_fr=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_VELOCITY_FEEDBACK)
        # time.sleep(0.0015)
        position_data_walk_vel_rl=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_VELOCITY_FEEDBACK)
        # time.sleep(0.0015)
        position_data_walk_vel_rr=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_VELOCITY_FEEDBACK)
        # time.sleep(0.0015)
        ret,kk=self.MobileControl.CanAnalysis.Can_New_Receive(0,RecNum)
        # time.sleep(0.03)
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
    def output_pulse_position_control_multi(self,flag_list,rad_fl,rad_fr,rad_rl,rad_rr):
        """
        flag_list=[]
        """
        oldpusle_fl=self.Driver_steer_encode_fl_original
        oldpusle_fr=self.Driver_steer_encode_fr_original
        oldpusle_rl=self.Driver_steer_encode_rl_original
        oldpusle_rr=self.Driver_steer_encode_rr_original
        return [oldpusle_fl+flag_list[0]*self.degree_to_pulse(rad_fl),oldpusle_fr+flag_list[1]*self.degree_to_pulse(rad_fr),oldpusle_rl+flag_list[2]*self.degree_to_pulse(rad_rl),oldpusle_rr+flag_list[3]*self.degree_to_pulse(rad_rr)]

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

def main():

    mpfh=MobilePlatFormCmdVelControl()


    mpfh.Init_Ros_Node()


    # mpfh.MobileControl.CanAnalysis.Can_ReadBoardInfo()
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0

    ratet=100

    rate = rospy.Rate(ratet)
    # while not rospy.is_shutdown():
    #     print "---bicycle_model-----",mpfh.caculate_bicycle_model_thetafr_re()
    #     print "----four_steer_degree----",mpfh.caculate_four_steer_degree_theta()
    #     print "-----four_walk velocity------",mpfh.caculate_four_walk_motor_velocity()
    #     rate.sleep()
    data_array=[]
    pubmsg=irr_encode_msg()

    flg=1
    # try:
    flag=1
    while not rospy.is_shutdown():
        if 0 not in [mpfh.Driver_steer_encode_fl_original,mpfh.Driver_steer_encode_fr_original,mpfh.Driver_steer_encode_rl_original,mpfh.Driver_steer_encode_rr_original]:
            if flg:
                mpfh.Init_mobile_driver()
                OutputPulse=mpfh.output_pulse_position_control_zero([.0,.0,.0,.0],0)
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
                    print "---bicycle_model-----",mpfh.caculate_bicycle_model_thetafr_re()
                    
                    
                    four_walk_velocity=mpfh.caculate_four_walk_motor_velocity()
                    # print "VelocityData:RPM/Min",VelocityData
                    print "-----four_walk velocity------",four_walk_velocity
                    VelocityData_fl= mpfh.caculate_velocity(1.0*four_walk_velocity[1])
                    VelocityData_fr= mpfh.caculate_velocity(1.0*four_walk_velocity[0])
                    VelocityData_rl= mpfh.caculate_velocity(1.0*four_walk_velocity[3])
                    VelocityData_rr= mpfh.caculate_velocity(1.0*four_walk_velocity[2])
                    mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData_fl),'left',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'])
                    mpfh.MobileControl.Send_Velocity_Driver(1*int(VelocityData_fr),'right',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'])
                    mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData_rl),'left',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'])
                    mpfh.MobileControl.Send_Velocity_Driver(1*int(VelocityData_rr),'right',mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'])
                    print "real velocity in rpm/min",[VelocityData_fl,VelocityData_fr,VelocityData_rl,VelocityData_rr]
                    time.sleep(0.001)
                    four_steer_rad=mpfh.caculate_four_steer_degree_theta()
                    print "----four_steer_degree----",four_steer_rad
                    OutputPulse=mpfh.output_pulse_position_control_multi([1.0,-1.0,1.0,-1.0],four_steer_rad[1],four_steer_rad[0],four_steer_rad[3],four_steer_rad[2])#output_pulse_position_control_zero([four_steer_rad[1],four_steer_rad[0],four_steer_rad[3],four_steer_rad[2]],turn)
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[0]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[1]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[2]),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[3]),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                    print "real position----",OutputPulse
        rate.sleep()            

    mpfh.MobileControl.CanAnalysis.Can_VCICloseDevice()
    # except:
    #     print "haha"
   
if __name__=="__main__":
    main()