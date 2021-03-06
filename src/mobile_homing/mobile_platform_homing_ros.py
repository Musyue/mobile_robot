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
from logger_config.logger_set import *
from config.command import *
from can_analysis.can_analysis_driver import *
import time
from mobile_control.mobileplatform_driver import *
import binascii
from pid_control.pid_control import *
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import spline

class MobilePlatFormHoming():
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

        self.Pid=PID()
        self.Driver_steer_encode_fl=0
        self.Driver_steer_encode_fr=0
        self.Driver_steer_encode_rl=0
        self.Driver_steer_encode_rr=0
        ####ROS
        self.close_homing_flag=True
        self.homing_ok_pub = rospy.Publisher("/mobile_platform_homing_status", Bool, queue_size=10)
        self.driver_position_feedback_pub = rospy.Publisher("/mobile_platform_driver_position_feedback", Int64MultiArray, queue_size=10)
        self.sub_close_homing_status=rospy.Subscriber('/close_homing_topic',Bool,self.close_homing_callback)
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

        # position_data_fl=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_POSITION_FEEDBACK)
        # time.sleep(0.0015)
        # position_data_fr=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_POSITION_FEEDBACK)
        # time.sleep(0.0015)
        # position_data_rl=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_POSITION_FEEDBACK)
        # time.sleep(0.0015)
        # position_data_rr=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_POSITION_FEEDBACK)
        # time.sleep(0.0015)

        ret,kk=self.MobileControl.CanAnalysis.Can_New_Receive(0,RecNum)
        time.sleep(0.03)
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
                # if list(kk[i].Data)[0]!=127:
                #     print "kk[i].ID",kk[i].ID
                    # print  "list(kk[i].Data)",list(kk[i].Data)
                # if (kk[i].ID==0x81 or kk[i].ID==0x82 or kk[i].ID==0x83 or kk[i].ID==0x84):
                #     print "------------------------------ermergecy code -------",list(kk[i].Data)
                #     a = input("input:")
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


    def Init_mobile_platform(self):
        self.MobileControl.Opreation_Controller_Mode(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.SET_MODE_VELOCITY)
        # time.sleep(0.1)
        self.MobileControl.Opreation_Controller_Mode(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.SET_MODE_VELOCITY)
        # time.sleep(0.1)
        self.MobileControl.Enable_Steering_Controller()
        # time.sleep(0.1)
    def Init_Pid_Control(self,P,I,D,windup,sampletime):
        self.Pid.setKp(P)
        self.Pid.setKi(I)
        self.Pid.setKd(D)
        self.Pid.setWindup(windup)
        self.Pid.setSampleTime(sampletime)    
        # self.MobileControl.Save_Parameter('steer')
    def Init_Ros_Node(self):
        rospy.init_node("homing_for_mobileplatform")

    

def main():

    windup=20
    sampletime=0.01
    mpfh=MobilePlatFormHoming()
    mpfh.Init_mobile_platform()
    # mpfh.Init_Pid_Control(P,I,D,windup,sampletime)
    mpfh.Init_Ros_Node()
    P=mpfh.MobileControl.CanAnalysis.yamlDic['Pid_parameter']['P']
    I=mpfh.MobileControl.CanAnalysis.yamlDic['Pid_parameter']['I']
    D=mpfh.MobileControl.CanAnalysis.yamlDic['Pid_parameter']['D']
    count=1
    flag=0
    recevenum=0
    flag_1=1
    count_num_fl=0
    count_num_fr=0
    count_num_rl=0
    count_num_rr=0
    flag_fl = 'fl'
    flag_fr= 'fr'
    flag_rl= 'rl'
    flag_rr= 'rr'
    END=100
    ratet=100
    homing_ok_dic={}
    ######fl
    output_fl=[]
    output_fr=[]
    output_rl=[]
    output_rr=[]
    feedback_fl=0
    feedback_list_fl = []
    time_list_fl = []
    setpoint_list_fl = []
    status_ok_fl=0
    #####fr
    feedback_fr=0
    feedback_list_fr = []
    time_list_fr = []
    setpoint_list_fr = []
    status_ok_fr=0
    #####rl
    feedback_rl=0
    feedback_list_rl = []
    time_list_rl = []
    setpoint_list_rl = []
    status_ok_rl=0
    #######rr
    feedback_rr=0
    feedback_list_rr = []
    time_list_rr = []
    setpoint_list_rr = []
    status_ok_rr=0
    #######
    rate = rospy.Rate(ratet)
    while not rospy.is_shutdown():
        # print "haha"
        recevenum=mpfh.MobileControl.CanAnalysis.Can_GetReceiveNum(0)
        # print recevenum
        # mpfh.New_Read_Encoder_data_From_ABS_Encoder(recevenum)
        if recevenum!=None:
            if mpfh.Abs_Encoder_fr_id2_oct!=0 and mpfh.Abs_Encoder_fl_id1_oct!=0 and mpfh.Abs_Encoder_rr_id4_oct!=0 and mpfh.Abs_Encoder_rl_id3_oct!=0:
                mpfh.New_Read_Encoder_data_From_ABS_Encoder(recevenum)
                print "-----Abs_Encoder_fr_id2_oct",mpfh.Abs_Encoder_fr_id2_oct
                print "-----Abs_Encoder_fl_id1_oct",mpfh.Abs_Encoder_fl_id1_oct
                print "-----Abs_Encoder_rl_id3_oct",mpfh.Abs_Encoder_rl_id3_oct
                print "-----Abs_Encoder_rr_id4_oct",mpfh.Abs_Encoder_rr_id4_oct
                if flag_1==1:
                    if flag_fl == 'fl':
                        fl_error=mpfh.fl_abs_encode-mpfh.Abs_Encoder_fl_id1_oct
                        print "-----fl error----",fl_error

                        output_fl.append(fl_error)

                        if len(output_fl)>3:
                            # print "output_fl",output_fl
                            velocity_control=P*(output_fl[count_num_fl]-output_fl[count_num_fl-1])+I*output_fl[count_num_fl]+D*(output_fl[count_num_fl]-2*output_fl[count_num_fl-1]+output_fl[count_num_fl-2])
                            out_vel=(velocity_control*60*50)/1024
                            print "out_vel-----fl",out_vel
                            if abs(out_vel)>1500 and out_vel<0:
                                mpfh.MobileControl.Send_Velocity_Driver(int(-1500),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                print "count num",count_num_fl
                            elif abs(out_vel)>1500 and out_vel>0:
                                mpfh.MobileControl.Send_Velocity_Driver(int(1500),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                print "count num",count_num_fl                          
                            else:
                                mpfh.MobileControl.Send_Velocity_Driver(int(out_vel),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                print "count num",count_num_fl
                        if abs(fl_error)<=mpfh.MobileControl.CanAnalysis.yamlDic['Homing_error_limit']:
                            status_ok_fl+=1
                            if status_ok_fl>=3:
                                mpfh.MobileControl.Send_Velocity_Driver(0,'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                flag_fl=0
                                # count_num_fl=0
                                output_fl=[]
                                homing_ok_dic.update({'fl':1})

                        count_num_fl+=1
                    if flag_fr == 'fr':
                        fr_error=mpfh.fr_abs_encode-mpfh.Abs_Encoder_fr_id2_oct
                        print "-----fr error----",fr_error

                        output_fr.append(fr_error)

                        if len(output_fr)>3:
                            velocity_control=P*(output_fr[count_num_fr]-output_fr[count_num_fr-1])+I*output_fr[count_num_fr]+D*(output_fr[count_num_fr]-2*output_fr[count_num_fr-1]+output_fr[count_num_fr-2])
                            out_vel=(velocity_control*60*50)/1024
                            print "out_vel-----fr",out_vel
                            if abs(out_vel)>1500 and out_vel<0:
                                mpfh.MobileControl.Send_Velocity_Driver(int(-1500),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                print "count num",count_num_fr
                            elif abs(out_vel)>1500 and out_vel>0:
                                mpfh.MobileControl.Send_Velocity_Driver(int(1500),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                print "count num",count_num_fr
                            else:
                                mpfh.MobileControl.Send_Velocity_Driver(int(out_vel),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                print "count num",count_num_fr

                        if abs(fr_error)<=mpfh.MobileControl.CanAnalysis.yamlDic['Homing_error_limit']:
                            status_ok_fr+=1
                            if status_ok_fr>=3:
                                mpfh.MobileControl.Send_Velocity_Driver(0,'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                                flag_fr=0
                                # count_num_fr=0
                                output_fr=[]
                                homing_ok_dic.update({'fr':1})

                        count_num_fr+=1
                    if flag_rl == 'rl':
                        rl_error=mpfh.rl_abs_encode-mpfh.Abs_Encoder_rl_id3_oct
                        print "-----rl error----",rl_error

                        output_rl.append(rl_error)

                        if len(output_rl)>3:
                            velocity_control=P*(output_rl[count_num_rl]-output_rl[count_num_rl-1])+I*output_rl[count_num_rl]+D*(output_rl[count_num_rl]-2*output_rl[count_num_rl-1]+output_rl[count_num_rl-2])
                            out_vel=(velocity_control*60*50)/1024
                            print "out_vel-----rl",out_vel
                            if abs(out_vel)>1500 and out_vel<0:
                                mpfh.MobileControl.Send_Velocity_Driver(int(-1500),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                                print "count num",count_num_rl
                            elif abs(out_vel)>1500 and out_vel>0:
                                mpfh.MobileControl.Send_Velocity_Driver(int(1500),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                                print "count num",count_num_rl
                            else:
                                mpfh.MobileControl.Send_Velocity_Driver(int(out_vel),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                                print "count num",count_num_rl

                        if abs(rl_error)<=mpfh.MobileControl.CanAnalysis.yamlDic['Homing_error_limit']:
                            status_ok_rl+=1
                            if status_ok_rl>=3:
                                mpfh.MobileControl.Send_Velocity_Driver(0,'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                                flag_rl=0
                                # count_num_rl=0
                                output_rl=[]
                                homing_ok_dic.update({'rl':1})

                        count_num_rl+=1
                    if flag_rr == 'rr':
                        rr_error=mpfh.rr_abs_encode-mpfh.Abs_Encoder_rr_id4_oct
                        print "-----rr error----",rr_error

                        output_rr.append(rr_error)

                        if len(output_rr)>3:
                            velocity_control=P*(output_rr[count_num_rr]-output_rr[count_num_rr-1])+I*output_rr[count_num_rr]+D*(output_rr[count_num_rr]-2*output_rr[count_num_rr-1]+output_rr[count_num_rr-2])
                            out_vel=(velocity_control*60*50)/1024
                            print "out_vel-----rr",out_vel
                            if abs(out_vel)>1500 and out_vel<0:
                                mpfh.MobileControl.Send_Velocity_Driver(int(-1500),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                                print "count num",count_num_rr
                            elif abs(out_vel)>1500 and out_vel>0:
                                mpfh.MobileControl.Send_Velocity_Driver(int(1500),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                                print "count num",count_num_rr
                            else:
                                mpfh.MobileControl.Send_Velocity_Driver(int(out_vel),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                                print "count num",count_num_rr

                        if abs(rr_error)<=mpfh.MobileControl.CanAnalysis.yamlDic['Homing_error_limit']+1:
                            status_ok_rr+=1
                            if status_ok_rr>=3:
                                mpfh.MobileControl.Send_Velocity_Driver(0,'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                                flag_rr=0
                                # count_num_rr=0
                                output_rr=[]
                                homing_ok_dic.update({'rr':1})

                        count_num_rr+=1
                if homing_ok_dic.has_key('fl') and homing_ok_dic.has_key('fr') and homing_ok_dic.has_key('rl') and homing_ok_dic.has_key('rr'):
                    if homing_ok_dic['fl']==1 and homing_ok_dic['fr']==1 and  homing_ok_dic['rl']==1 and  homing_ok_dic['rr']==1:  
                        flag_1=0
                        mpfh.MobileControl.Save_Parameter(1)
                        mpfh.MobileControl.Disable_ALL_Motor_Controller()
                        print "---------------homing is ok------------------------"
                        time.sleep(3)
                        mpfh.MobileControl.Save_Parameter(1)
                        mpfh.MobileControl.CanAnalysis.Can_VCICloseDevice()
                        print "---------------homing is ok------------------------"

                        status_ok_fl=0
                        status_ok_fr=0
                        status_ok_rl=0
                        status_ok_rr=0
                        # if mpfh.close_homing_flag:
                        #     mpfh.MobileControl.CanAnalysis.Can_VCICloseDevice()
                        #     pubarray=[mpfh.Driver_steer_encode_fl,mpfh.Driver_steer_encode_fr,mpfh.Driver_steer_encode_rl,mpfh.Driver_steer_encode_rr]
                        #     positiondata_array_for_publishing = Int64MultiArray(data=pubarray)
                        #     mpfh.homing_ok_pub.publish(True)
                        #     mpfh.driver_position_feedback_pub.publish(positiondata_array_for_publishing)
            else:
                mpfh.New_Read_Encoder_data_From_ABS_Encoder(recevenum)
                print "read data",recevenum
        if homing_ok_dic.has_key('fl') and homing_ok_dic.has_key('fr') and homing_ok_dic.has_key('rl') and homing_ok_dic.has_key('rr'):
            if homing_ok_dic['fl']==1 and homing_ok_dic['fr']==1 and  homing_ok_dic['rl']==1 and  homing_ok_dic['rr']==1:  
                
                if mpfh.close_homing_flag:
                    # print "---------------homing is ok------------------------"
                    # rospy.logerr("---------------homing is ok------------------------")
                    # mpfh.MobileControl.CanAnalysis.Can_VCICloseDevice()
                    pubarray=[mpfh.Driver_steer_encode_fl,mpfh.Driver_steer_encode_fr,mpfh.Driver_steer_encode_rl,mpfh.Driver_steer_encode_rr]
                    positiondata_array_for_publishing = Int64MultiArray(data=pubarray)
                    mpfh.homing_ok_pub.publish(True)
                    mpfh.driver_position_feedback_pub.publish(positiondata_array_for_publishing)
                        # time.sleep(0.1)
                elif mpfh.close_homing_flag==False:
                    print "close uploading data to topics-------------"
                else:
                    pass
        rate.sleep()            
   
if __name__=="__main__":
    main()