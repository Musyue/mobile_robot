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


class MobilePlatFormSensordata():
    def __init__(self,):
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
        ####ROS


        self.driver_position_feedback_pub = rospy.Publisher("/mobile_platform_driver_sensor_feedback", Int64MultiArray, queue_size=10)

    def close_homing_callback(self,msg):
        # print msg.data
        self.close_homing_flag=msg.data
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
        transmit_status_1=self.MobileControl.CanAnalysis.Can_Transmit(0,1,1,8,self.MobileControl.MobileDriver_Command.REQUEST_ENCODER_1)
        time.sleep(0.0015)
        transmit_status_2=self.MobileControl.CanAnalysis.Can_Transmit(0,1,2,8,self.MobileControl.MobileDriver_Command.REQUEST_ENCODER_2)
        time.sleep(0.0015)
        transmit_status_3=self.MobileControl.CanAnalysis.Can_Transmit(0,1,3,8,self.MobileControl.MobileDriver_Command.REQUEST_ENCODER_3)
        time.sleep(0.0015)
        transmit_status_4=self.MobileControl.CanAnalysis.Can_Transmit(0,1,4,8,self.MobileControl.MobileDriver_Command.REQUEST_ENCODER_4)
        time.sleep(0.0015)
        #position feedback
        position_data_fl=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_POSITION_FEEDBACK)
        time.sleep(0.0015)
        position_data_fr=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_POSITION_FEEDBACK)
        time.sleep(0.0015)
        position_data_rl=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_POSITION_FEEDBACK)
        time.sleep(0.0015)
        position_data_rr=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_POSITION_FEEDBACK)
        time.sleep(0.0015)

        position_data_walk_fl=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_POSITION_FEEDBACK)
        time.sleep(0.0015)
        position_data_walk_fr=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_POSITION_FEEDBACK)
        time.sleep(0.0015)
        position_data_walk_rl=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_POSITION_FEEDBACK)
        time.sleep(0.0015)
        position_data_walk_rr=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_POSITION_FEEDBACK)
        time.sleep(0.0015)
        position_data_walk_vel_fl=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_VELOCITY_FEEDBACK)
        time.sleep(0.0015)
        position_data_walk_vel_fr=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_VELOCITY_FEEDBACK)
        time.sleep(0.0015)
        position_data_walk_vel_rl=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_VELOCITY_FEEDBACK)
        time.sleep(0.0015)
        position_data_walk_vel_rr=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['walking_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_VELOCITY_FEEDBACK)
        time.sleep(0.0015)
        ret,kk=self.MobileControl.CanAnalysis.Can_New_Receive(0,RecNum)
        if ret:
            for i in range(RecNum):
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and list(kk[i].Data)[1]==1:
                    # print('abs encoder 1',self.List_to_HEXList(list(kk[i].Data)))
                    # print(kk[i].DataLen)
                    # self.Abs_Encoder_fl_id1.append(self.List_to_HEXList(list(kk[i].Data)))
                    self.Abs_Encoder_fl_id1_oct=self.HEX_String_List_To_Oct(list(kk[i].Data))
                    self.MobileControl.logger.loggerinfo(self.HEX_String_List_To_Oct(list(kk[i].Data)))
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and list(kk[i].Data)[1]==2:
                    # print('abs encoder 2',self.List_to_HEXList(list(kk[i].Data)))
                    # print(kk[i].DataLen)
                    # self.Abs_Encoder_fr_id2.append(self.List_to_HEXList(list(kk[i].Data)))
                    self.Abs_Encoder_fr_id2_oct=self.HEX_String_List_To_Oct(list(kk[i].Data))
                    self.MobileControl.logger.loggerinfo(self.HEX_String_List_To_Oct(list(kk[i].Data)))
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and list(kk[i].Data)[1]==3:
                    # print('abs Encoder 3',self.List_to_HEXList(list(kk[i].Data)))
                    # print(kk[i].DataLen)
                    # self.Abs_Encoder_rl_id3.append(self.List_to_HEXList(list(kk[i].Data)))
                    self.MobileControl.logger.loggerinfo(self.HEX_String_List_To_Oct(list(kk[i].Data)))
                    self.Abs_Encoder_rl_id3_oct=self.HEX_String_List_To_Oct(list(kk[i].Data))
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and list(kk[i].Data)[1]==4:
                    # print('abs Encoder 4',self.List_to_HEXList(list(kk[i].Data)))
                    # print(kk[i].DataLen)
                    # self.Abs_Encoder_rr_id4.append(self.List_to_HEXList(list(kk[i].Data)))
                    
                    self.Abs_Encoder_rr_id4_oct=self.HEX_String_List_To_Oct(list(kk[i].Data))

                    self.MobileControl.logger.loggerinfo(self.HEX_String_List_To_Oct(list(kk[i].Data)))
                # print "kk[i].ID",kk[i].ID
                # print  "list(kk[i].Data)",list(kk[i].Data)
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x583 and list(kk[i].Data)[2]==96 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    print('driver position encode fl',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_steer_encode_fl=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))
                    print "self.Driver_steer_encode_fl",self.Driver_steer_encode_fl
                    # print "list(kk[i].Data)",list(kk[i].Data)
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x583 and list(kk[i].Data)[2]==104 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    print('driver position encode fr',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_steer_encode_fr=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))
                    print " self.Driver_steer_encode_fr", self.Driver_steer_encode_fr
                    # print "list(kk[i].Data)",list(kk[i].Data)
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x584 and list(kk[i].Data)[2]==96 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    print('driver position encode rl',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_steer_encode_rl=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))
                    print 'self.Driver_steer_encode_rl',self.Driver_steer_encode_rl
                    # print "list(kk[i].Data)",list(kk[i].Data)
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x584 and list(kk[i].Data)[2]==104 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    print('driver position encode rr',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_steer_encode_rr=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))         
                    print "self.Driver_steer_encode_rr",self.Driver_steer_encode_rr
                    # print "list(kk[i].Data)",list(kk[i].Data)
                ###walk
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x581 and list(kk[i].Data)[1]==100 and list(kk[i].Data)[2]==96 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    print('driver position walk encode fl',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_walk_encode_fl=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))
                    print "self.Driver_walk_encode_fl",self.Driver_walk_encode_fl
                    # print "list(kk[i].Data)",list(kk[i].Data)
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x581 and list(kk[i].Data)[1]==100 and list(kk[i].Data)[2]==104 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    print('driver position encode fr',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_walk_encode_fr=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))
                    print " self.Driver_walk_encode_fr", self.Driver_walk_encode_fr
                    # print "list(kk[i].Data)",list(kk[i].Data)
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x582 and list(kk[i].Data)[1]==100 and list(kk[i].Data)[2]==96 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    print('driver position encode rl',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_walk_encode_rl=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))
                    print 'self.Driver_walk_encode_rl',self.Driver_walk_encode_rl
                    # print "list(kk[i].Data)",list(kk[i].Data)
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x582 and list(kk[i].Data)[1]==100 and list(kk[i].Data)[2]==104 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    print('driver position encode rr',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_walk_encode_rr=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))         
                    print "self.Driver_walk_encode_rr",self.Driver_walk_encode_rr
                    # print "list(kk[i].Data)",list(kk[i].Data)
                ###walk velocity

                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x581 and list(kk[i].Data)[1]==105 and list(kk[i].Data)[2]==96 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    print('driver velocity walk encode fl',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_walk_velocity_encode_fl=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))
                    print "self.Driver_walk_velocity_encode_fl",self.Driver_walk_velocity_encode_fl
                    # print "list(kk[i].Data)",list(kk[i].Data)
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x581 and list(kk[i].Data)[1]==105 and list(kk[i].Data)[2]==104 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    print('driver velocity encode fr',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_walk_velocity_encode_fr=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))
                    print " self.Driver_walk_velocity_encode_fr", self.Driver_walk_velocity_encode_fr
                    # print "list(kk[i].Data)",list(kk[i].Data)
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x582 and list(kk[i].Data)[1]==105 and list(kk[i].Data)[2]==96 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    print('driver velocity encode rl',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_walk_velocity_encode_rl=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))
                    print 'self.Driver_walk_velocity_encode_rl',self.Driver_walk_velocity_encode_rl
                    # print "list(kk[i].Data)",list(kk[i].Data)
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and kk[i].ID==0x582 and list(kk[i].Data)[1]==105 and list(kk[i].Data)[2]==104 and list(kk[i].Data)[7]!=0 and list(kk[i].Data)[6]!=0 and list(kk[i].Data)[5]!=0 and list(kk[i].Data)[4]!=0:
                    print('driver velocity encode rr',self.List_to_HEXList(list(kk[i].Data)))
                    self.Driver_walk_velocity_encode_rr=self.HEX_String_List_To_Oct_Four(list(kk[i].Data))         
                    print "self.Driver_walk_velocity_encode_rr",self.Driver_walk_velocity_encode_rr
                    # print "list(kk[i].Data)",list(kk[i].Data)

    def Init_Ros_Node(self):
        rospy.init_node("get_driverencode_for_mobileplatform")

def main():


    mpfh=MobilePlatFormSensordata()


    mpfh.Init_Ros_Node()
']
    count=0
    flag=0
    recevenum=0
    flag_1=1

    ratet=1

    rate = rospy.Rate(ratet)
    data_array=[]
    pubmsg=irr_encode_msg()
    while not rospy.is_shutdown():
        
        # print "haha"
        recevenum=mpfh.MobileControl.CanAnalysis.Can_GetReceiveNum(0)
        # print recevenum
        # mpfh.New_Read_Encoder_data_From_ABS_Encoder(recevenum)
        if recevenum!=None:
            data_array=[
            mpfh.Abs_Encoder_fl_id1_oct,mpfh.Abs_Encoder_fl_id1_oct,mpfh.Abs_Encoder_rr_id4_oct,mpfh.Driver_walk_encode_fl,
            mpfh.Driver_walk_encode_fr,mpfh.Driver_walk_encode_rl,mpfh.Driver_walk_encode_rr,mpfh.Driver_steer_encode_fl,mpfh.Driver_steer_encode_fr
            mpfh.Driver_steer_encode_rl,mpfh.Driver_steer_encode_rr,mpfh.Driver_walk_velocity_encode_fl,mpfh.Driver_walk_velocity_encode_fr,
            mpfh.Driver_walk_velocity_encode_rl,mpfh.Driver_walk_velocity_encode_rr
            
            ]
            if 0 not in data_array:
                mpfh.New_Read_Encoder_data_From_ABS_Encoder(recevenum)
                rospy.loginfo("-----Abs_Encoder_fl_id1_oct----"+str(mpfh.Abs_Encoder_fl_id1_oct))
                rospy.loginfo("-----Abs_Encoder_fr_id2_oct----"+str(mpfh.Abs_Encoder_fr_id2_oct))
                rospy.loginfo("-----Abs_Encoder_rl_id3_oct----"+str(mpfh.Abs_Encoder_rl_id3_oct))
                rospy.loginfo("-----Abs_Encoder_rr_id4_oct----"+str(mpfh.Abs_Encoder_rr_id4_oct))

                rospy.loginfo("-----Driver_walk_encode_fl----"+str(mpfh.Driver_walk_encode_fl))
                rospy.loginfo("-----Driver_walk_encode_fr----"+str(mpfh.Driver_walk_encode_fr))
                rospy.loginfo("-----Driver_walk_encode_rl----"+str(mpfh.Driver_walk_encode_rl))
                rospy.loginfo("-----Driver_walk_encode_rr----"+str(mpfh.Driver_walk_encode_rr))
                rospy.loginfo("-----Driver_walk_velocity_encode_fl----"+str(mpfh.Driver_walk_velocity_encode_fl))
                rospy.loginfo("-----Driver_walk_velocity_encode_fr----"+str(mpfh.Driver_walk_velocity_encode_fr))
                rospy.loginfo("-----Driver_walk_velocity_encode_rl----"+str(mpfh.Driver_walk_velocity_encode_rl))
                rospy.loginfo("-----Driver_walk_velocity_encode_rr----"+str(mpfh.Driver_walk_velocity_encode_rr))
                rospy.loginfo("-----Driver_steer_encode_fl----"+str(mpfh.Driver_steer_encode_fl))
                rospy.loginfo("-----Driver_steer_encode_fr----"+str(mpfh.Driver_steer_encode_fr))
                rospy.loginfo("-----Driver_steer_encode_rl----"+str(mpfh.Driver_steer_encode_rl))
                rospy.loginfo("-----Driver_steer_encode_rr----"+str(mpfh.Driver_steer_encode_rr))
                pubmsg.Count=count
                pubmsg.Abs_Encoder_fl_id1_oct=mpfh.Abs_Encoder_fl_id1_oct
                pubmsg.Abs_Encoder_fr_id2_oct=mpfh.Abs_Encoder_fr_id2_oct
                pubmsg.Abs_Encoder_rl_id3_oct=mpfh.Abs_Encoder_rl_id3_oct
                pubmsg.Abs_Encoder_rr_id4_oct=mpfh.Abs_Encoder_rr_id4_oct
                pubmsg.Driver_steer_encode_fl=mpfh.Driver_steer_encode_fl
                pubmsg.Driver_steer_encode_fr=mpfh.Driver_steer_encode_fr
                pubmsg.Driver_steer_encode_rl=mpfh.Driver_steer_encode_rl
                pubmsg.Driver_steer_encode_rr=mpfh.Driver_steer_encode_rr
                pubmsg.Driver_walk_encode_fl=mpfh.Driver_walk_encode_fl
                pubmsg.Driver_walk_encode_fr=mpfh.Driver_walk_encode_fr
                pubmsg.Driver_walk_encode_rl=mpfh.Driver_walk_encode_rl
                pubmsg.Driver_walk_encode_rr=mpfh.Driver_walk_encode_rr
                pubmsg.Driver_walk_velocity_encode_fl=mpfh.Driver_walk_velocity_encode_fl
                pubmsg.Driver_walk_velocity_encode_fr=mpfh.Driver_walk_velocity_encode_fr
                pubmsg.Driver_walk_velocity_encode_rl=mpfh.Driver_walk_velocity_encode_rl
                pubmsg.Driver_walk_velocity_encode_rr=mpfh.Driver_walk_velocity_encode_rr
                mpfh.driver_position_feedback_pub.publish(pubmsg)
                count+=1
        else:
            mpfh.New_Read_Encoder_data_From_ABS_Encoder(recevenum)
        rate.sleep()            
   
if __name__=="__main__":
    main()