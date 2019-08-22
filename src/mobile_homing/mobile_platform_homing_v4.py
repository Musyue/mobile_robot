#! /usr/bin/env python
# coding=utf-8
import sys
sys.path.append("..")
from ctypes import *
import yaml
import os
from math import pi
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
        # self.MobileControl.Enable_Steering_Controller()#初始化四个舵轮的两个驱动器
        self.Abs_Encoder_fl_id1=[]
        self.Abs_Encoder_fr_id2=[]
        self.Abs_Encoder_rl_id3=[]
        self.Abs_Encoder_rr_id4=[]
        self.Abs_Encoder_fl_id1_oct=0
        self.Abs_Encoder_fr_id2_oct=0
        self.Abs_Encoder_rl_id3_oct=0
        self.Abs_Encoder_rr_id4_oct=0
        self.fl_abs_encode=self.MobileControl.CanAnalysis.yamlDic['Homing_abs_encoder_data']['fl']
        self.fr_abs_encode=self.MobileControl.CanAnalysis.yamlDic['Homing_abs_encoder_data']['fr']
        self.rl_abs_encode=self.MobileControl.CanAnalysis.yamlDic['Homing_abs_encoder_data']['rl']
        self.rr_abs_encode=self.MobileControl.CanAnalysis.yamlDic['Homing_abs_encoder_data']['rr']
        self.limit_error=self.MobileControl.CanAnalysis.yamlDic['Homing_error_limit']
        self.home_ok_flag={}
        self.Pid=PID()
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
    def New_Read_Encoder_data_From_ABS_Encoder(self,RecNum):
        transmit_status_1=self.MobileControl.CanAnalysis.Can_Transmit(0,1,1,8,self.MobileControl.MobileDriver_Command.REQUEST_ENCODER_1)
        time.sleep(0.05)
        transmit_status_2=self.MobileControl.CanAnalysis.Can_Transmit(0,1,2,8,self.MobileControl.MobileDriver_Command.REQUEST_ENCODER_2)
        time.sleep(0.05)
        transmit_status_3=self.MobileControl.CanAnalysis.Can_Transmit(0,1,3,8,self.MobileControl.MobileDriver_Command.REQUEST_ENCODER_3)
        time.sleep(0.05)
        transmit_status_4=self.MobileControl.CanAnalysis.Can_Transmit(0,1,4,8,self.MobileControl.MobileDriver_Command.REQUEST_ENCODER_4)
        time.sleep(0.05)
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
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127and list(kk[i].Data)[1]==4:
                    # print('abs Encoder 4',self.List_to_HEXList(list(kk[i].Data)))
                    # print(kk[i].DataLen)
                    # self.Abs_Encoder_rr_id4.append(self.List_to_HEXList(list(kk[i].Data)))
                    self.Abs_Encoder_rr_id4_oct=self.HEX_String_List_To_Oct(list(kk[i].Data))
                    self.MobileControl.logger.loggerinfo(self.HEX_String_List_To_Oct(list(kk[i].Data)))             

    def Run_Four_Steer_Motor(self,flag):#in 200rpm
        if flag=="steer_chn2_left":
            self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_TEST_200_VELOCITY_COMMAND)
            time.sleep(0.01)
        elif flag=="steer_chn2_right":
            self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_TEST_200_VELOCITY_COMMAND)
            time.sleep(0.01)
        elif flag== "steer_chn1_left":
            self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_TEST_200_VELOCITY_COMMAND)
            time.sleep(0.01)
        elif flag=="steer_chn1_right":
            self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_TEST_200_VELOCITY_COMMAND)
            time.sleep(0.01)
        # self.MobileControl.logger.loggerinfo("---!!!!!--RUN ALL STEER Motor in 200 RPM/min-----!!!!","LIGHT_RED")
        elif flag=="all":
            self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_TEST_200_VELOCITY_COMMAND)
            time.sleep(0.01)
            
            self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_TEST_200_VELOCITY_COMMAND)
            time.sleep(0.01)
            self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_TEST_200_VELOCITY_COMMAND)
            time.sleep(0.01)
            self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_TEST_200_VELOCITY_COMMAND)
            time.sleep(0.01)
            self.MobileControl.logger.loggerinfo("---!!!!!--RUN ALL STEER Motor in 200 RPM/min-----!!!!","LIGHT_RED")
        else:
            self.MobileControl.logger.loggererror("Flag error!!Please check ")
    def Stop_Four_Steer_Motor(self,flag):
        if flag=="steer_chn2_left":
            self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_TARGET_VELOCITY_COMMAND)
            time.sleep(0.1)
        elif flag=="steer_chn2_right":
            self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_TARGET_VELOCITY_COMMAND)
            time.sleep(0.1)
        elif flag=="steer_chn1_left":
            self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_TARGET_VELOCITY_COMMAND)
            time.sleep(0.1)
        elif flag=="steer_chn1_right":
            self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_TARGET_VELOCITY_COMMAND)
            time.sleep(0.1)
            # self.MobileControl.logger.loggerinfo("!!!!----STOP ALL STEER Motor in 200 RPM/min !!!!!----","LIGHT_RED")

        elif flag=="all":
            self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_TARGET_VELOCITY_COMMAND)
            time.sleep(0.1)
            self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_TARGET_VELOCITY_COMMAND)
            time.sleep(0.1)
            self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_TARGET_VELOCITY_COMMAND)
            time.sleep(0.1)
            self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_TARGET_VELOCITY_COMMAND)
            time.sleep(0.1)
            self.MobileControl.logger.loggerinfo("!!!!----STOP ALL STEER Motor in 200 RPM/min !!!!!----","LIGHT_RED")
        else:
            self.MobileControl.logger.loggererror("Flag error!!Please check ")
    def Init_mobile_platform(self):
        self.MobileControl.Opreation_Controller_Mode(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.SET_MODE_VELOCITY)
        time.sleep(0.1)
        self.MobileControl.Opreation_Controller_Mode(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.SET_MODE_VELOCITY)
        time.sleep(0.1)
        self.MobileControl.Enable_Steering_Controller()
        time.sleep(0.1)
    def Init_Pid_Control(self,P,I,D,windup,sampletime):
        self.Pid.setKp(P)
        self.Pid.setKi(I)
        self.Pid.setKd(D)
        self.Pid.setWindup(windup)
        self.Pid.setSampleTime(sampletime)    
        # self.MobileControl.Save_Parameter('steer')
    def Run_Four_Steer_Motor_New(self):
        if self.Abs_Encoder_fl_id1_oct!=0:
            # self.MobileControl.logger.loggererror('steer_chn1_left'+'-------->'+str(self.fl_abs_encode)+'---------'+str(self.Abs_Encoder_fl_id1_oct)+'--------------->'+str(abs(self.fl_abs_encode-self.Abs_Encoder_fl_id1_oct)))
            if self.fl_abs_encode-self.Abs_Encoder_fl_id1_oct <=0:
                self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_TEST_NEG_200_VELOCITY_COMMAND)
                time.sleep(0.1)
            else:
                self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_TEST_200_VELOCITY_COMMAND)
                time.sleep(0.1)
        else:
            pass

        if self.Abs_Encoder_fr_id2_oct!=0:
             # self.MobileControl.logger.loggererror('steer_chn1_left'+'-------->'+str(self.fl_abs_encode)+'---------'+str(self.Abs_Encoder_fl_id1_oct)+'--------------->'+str(abs(self.fl_abs_encode-self.Abs_Encoder_fl_id1_oct)))
            if self.fr_abs_encode-self.Abs_Encoder_fr_id2_oct <=0:
                self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_TEST_NEG_200_VELOCITY_COMMAND)
                time.sleep(0.1)
            else:
                self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_TEST_200_VELOCITY_COMMAND)
                time.sleep(0.1)
        else:
            pass

        if self.Abs_Encoder_rl_id3_oct!=0:
            # self.MobileControl.logger.loggererror('steer_chn1_left'+'-------->'+str(self.fl_abs_encode)+'---------'+str(self.Abs_Encoder_fl_id1_oct)+'--------------->'+str(abs(self.fl_abs_encode-self.Abs_Encoder_fl_id1_oct)))
            if self.rl_abs_encode-self.Abs_Encoder_rl_id3_oct <=0:
                self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_TEST_NEG_200_VELOCITY_COMMAND)
                time.sleep(0.1)
            else:
                self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_LEFT_TEST_200_VELOCITY_COMMAND)
                time.sleep(0.1)
        else:
            pass

        if self.Abs_Encoder_rr_id4_oct!=0:
            # self.MobileControl.logger.loggererror('steer_chn1_left'+'-------->'+str(self.fl_abs_encode)+'---------'+str(self.Abs_Encoder_fl_id1_oct)+'--------------->'+str(abs(self.fl_abs_encode-self.Abs_Encoder_fl_id1_oct)))
            if self.rr_abs_encode-self.Abs_Encoder_rr_id4_oct <=0:
                self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_TEST_NEG_200_VELOCITY_COMMAND)
                time.sleep(0.1)
            else:
                self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileControl.MobileDriver_Command.BASIC_RIGHT_TEST_200_VELOCITY_COMMAND)
                time.sleep(0.1)
        else:
            pass
    
    def Homing(self,vel):
        if self.Abs_Encoder_fl_id1_oct!=0:
            self.MobileControl.logger.loggererror('steer_chn1_left'+'-------->'+str(self.fl_abs_encode)+'---------'+str(self.Abs_Encoder_fl_id1_oct)+'--------------->'+str(abs(self.fl_abs_encode-self.Abs_Encoder_fl_id1_oct)))
            
            if abs(self.fl_abs_encode-self.Abs_Encoder_fl_id1_oct)<=self.limit_error:
                self.Stop_Four_Steer_Motor('steer_chn1_left')
                self.MobileControl.logger.loggererror("steer homing fl ok!!!!!")
                self.home_ok_flag.update({'steer_chn1_left':1})
    def Homing_1(self):
        if self.Abs_Encoder_fl_id1_oct!=0:
            self.MobileControl.logger.loggererror('steer_chn1_left'+'-------->'+str(self.fl_abs_encode)+'---------'+str(self.Abs_Encoder_fl_id1_oct)+'--------------->'+str(abs(self.fl_abs_encode-self.Abs_Encoder_fl_id1_oct)))
            if abs(self.fl_abs_encode-self.Abs_Encoder_fl_id1_oct)<=self.limit_error:
                self.Stop_Four_Steer_Motor('steer_chn1_left')
                self.MobileControl.logger.loggerinfo("steer homing fl ok!!!!!","LIGHT_RED")
                self.home_ok_flag.update({'steer_chn1_left':1})
        else:
            self.MobileControl.logger.loggerinfo("please wait encoder 1 feedback data","LIGHT_RED")

        if self.Abs_Encoder_fr_id2_oct!=0:
            self.MobileControl.logger.loggererror('steer_chn1_right'+'-------->'+str(self.fr_abs_encode)+'---------'+str(self.Abs_Encoder_fr_id2_oct)+'--------------->'+str(abs(self.fr_abs_encode-self.Abs_Encoder_fr_id2_oct)))
            # self.MobileControl.logger.loggerinfo('self.fr_abs_encode-self.Abs_Encoder_fr_id2_oct)-------->'+str(abs(self.fr_abs_encode-self.Abs_Encoder_fr_id2_oct)),"LIGHT_RED")
            if abs(self.fr_abs_encode-self.Abs_Encoder_fr_id2_oct)<=self.limit_error:
                self.Stop_Four_Steer_Motor('steer_chn1_right')
                self.MobileControl.logger.loggerinfo("steer homing fr ok!!!!!","LIGHT_RED")
                self.home_ok_flag.update({'steer_chn1_right':1})
        else:
            self.MobileControl.logger.loggerinfo("please wait encoder 2 feedback data","LIGHT_RED")

        if self.Abs_Encoder_rl_id3_oct!=0:
            self.MobileControl.logger.loggererror('steer_chn2_left'+'-------->'+str(self.rl_abs_encode)+'---------'+str(self.Abs_Encoder_rl_id3_oct)+'--------------->'+str(abs(self.rl_abs_encode-self.Abs_Encoder_rl_id3_oct)))
            # self.MobileControl.logger.loggerinfo('self.rl_abs_encode-self.Abs_Encoder_rl_id3_oct)------->'+str(abs(self.rl_abs_encode-self.Abs_Encoder_rl_id3_oct)),"LIGHT_RED")
            if abs(self.rl_abs_encode-self.Abs_Encoder_rl_id3_oct)<=self.limit_error:
                self.Stop_Four_Steer_Motor('steer_chn2_left')
                self.MobileControl.logger.loggerinfo("steer homing rl ok!!!!!","LIGHT_RED")
                self.home_ok_flag.update({'steer_chn2_left':1})
        else:
            self.MobileControl.logger.loggerinfo("please wait encoder 3 feedback data","LIGHT_RED")

        if self.Abs_Encoder_rr_id4_oct!=0:
            self.MobileControl.logger.loggererror('steer_chn2_right'+'-------->'+str(self.rr_abs_encode)+'---------'+str(self.Abs_Encoder_rr_id4_oct)+'--------------->'+str(abs(self.rr_abs_encode-self.Abs_Encoder_rr_id4_oct)))
            # self.MobileControl.logger.loggerinfo('self.rr_abs_encode-self.Abs_Encoder_rr_id4_oct)-------->'+str(abs(self.rr_abs_encode-self.Abs_Encoder_rr_id4_oct)),"LIGHT_RED")
            if abs(self.rr_abs_encode-self.Abs_Encoder_rr_id4_oct)<=self.limit_error:
                self.Stop_Four_Steer_Motor('steer_chn2_right')
                self.MobileControl.logger.loggerinfo("steer homing rr ok!!!!!","LIGHT_RED")
                self.home_ok_flag.update({'steer_chn2_right':1})
        else:
            self.MobileControl.logger.loggerinfo("please wait encoder 4 feedback data","LIGHT_RED")
            # self.MobileControl.logger.loggerinfo("Please wait homing progress!!!")
        # return home_ok_flag
    def Pid_control_Performance(self,feedback_list,setpoint_list,time_list,figure_num,END):
        time_sm = np.array(time_list)
        time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)
        feedback_smooth = spline(time_list, feedback_list, time_smooth)
        plt.figure(figure_num)
        plt.plot(time_smooth, feedback_smooth)
        plt.plot(time_list, setpoint_list)
        plt.xlim((0, END))
        plt.ylim((min(feedback_list)-0.5, max(feedback_list)+0.5))
        plt.xlabel('time (s)')
        plt.ylabel('PID (PV)')
        plt.title(str(figure_num)+'TEST PID')

        plt.ylim((1-60000.5, 1+60000.5))
        # plt.ylim((1-0.5, 1+0.5))
        plt.grid(True)
        plt.show()
def main():
    P=0.8#0.8
    I=0.1
    D=0.001
    windup=20
    sampletime=0.01
    mpfh=MobilePlatFormHoming()
    mpfh.Init_mobile_platform()
    mpfh.Init_Pid_Control(P,I,D,windup,sampletime)
    count=1
    flag=0
    recevenum=0
    flag_1=1
    count_num_fl=0
    count_num_fr=0
    count_num_rl=0
    count_num_rr=0
    flg_fl = 'fl'
    flg_fr='fr'
    flg_rl='rl'
    flg_rr='rr'
    END=100
    ######fl
    output_fl=[]
    output_fr=[]
    output_rl=[]
    output_rr=[]
    feedback_fl=0
    feedback_list_fl = []
    time_list_fl = []
    setpoint_list_fl = []
    #####fr
    feedback_fr=0
    feedback_list_fr = []
    time_list_fr = []
    setpoint_list_fr = []
    #####rl
    feedback_rl=0
    feedback_list_rl = []
    time_list_rl = []
    setpoint_list_rl = []
    #######rr
    feedback_rr=0
    feedback_list_rr = []
    time_list_rr = []
    setpoint_list_rr = []
    #######
    if flag:
        mpfh.Stop_Four_Steer_Motor('all')
    else:
        for i in range(1, END):
            recevenum=mpfh.MobileControl.CanAnalysis.Can_GetReceiveNum(0)
            # print("RECENUM------->",recevenum)
            print 'i',i
            if recevenum!=None:
                if flag_1==1 and mpfh.Abs_Encoder_fr_id2_oct!=0 and mpfh.Abs_Encoder_fl_id1_oct!=0 and mpfh.Abs_Encoder_rr_id4_oct!=0 and mpfh.Abs_Encoder_rl_id3_oct!=0:
                    mpfh.New_Read_Encoder_data_From_ABS_Encoder(recevenum)
                    
                    # time.sleep(0.01)
                    print "-----Abs_Encoder_fr_id2_oct",mpfh.Abs_Encoder_fr_id2_oct
                    print "-----Abs_Encoder_fl_id1_oct",mpfh.Abs_Encoder_fl_id1_oct
                    print "-----Abs_Encoder_rl_id3_oct",mpfh.Abs_Encoder_rl_id3_oct
                    print "-----Abs_Encoder_rr_id4_oct",mpfh.Abs_Encoder_rr_id4_oct
                    # if mpfh.rl_abs_encode-mpfh.Abs_Encoder_rl_id3_oct<0:
                    if flg_fl == 'fl':
                        mpfh.Pid.SetPoint = mpfh.fl_abs_encode
                        mpfh.Pid.update(feedback_fl)
                        output = mpfh.Pid.output
                        output_fl.append(output)
                        print "output",output
                        
                        print "------fl_abs_encode",mpfh.Pid.SetPoint
                        if mpfh.Pid.SetPoint > 0:
                            feedback_fl =mpfh.Abs_Encoder_fl_id1_oct
                            print "Feedback",feedback_fl
                        # if i>1:
                        #     mpfh.Pid.SetPoint = mpfh.fl_abs_encode
                        #     print "------fl_abs_encode",mpfh.Pid.SetPoint,mpfh.fl_abs_encode
                        if len(output_fl)>3:
                            velocity_control=mpfh.Pid.Kp*(output_fl[count_num_fl]-output_fl[count_num_fl-1])+mpfh.Pid.Ki*output_fl[count_num_fl]+mpfh.Pid.Kd*(output_fl[count_num_fl]-2*output_fl[count_num_fl-1]+output_fl[count_num_fl-2])
                            out_vel=(velocity_control*60*50)/1024
                            print "out_vel-----fl",out_vel
                            mpfh.MobileControl.Send_Velocity_Driver(int(out_vel),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                            print "count num",count_num_fl
                            # time.sleep(0.01)
                        feedback_list_fl.append(feedback_fl)
                        setpoint_list_fl.append(mpfh.Pid.SetPoint)
                        time_list_fl.append(i)
                        if abs(mpfh.Pid.Error)<=1:
                            mpfh.MobileControl.Send_Velocity_Driver(0,'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                            flg_fl=0
                            # count_num_fl=0
                            output_fl=[]

                        count_num_fl+=1
                    if flg_fr == 'fr':
                        mpfh.Pid.SetPoint = mpfh.fr_abs_encode
                        mpfh.Pid.update(feedback_fr)
                        output = mpfh.Pid.output
                        output_fr.append(output)
                        print "output",output
                        
                        print "------fr_abs_encode",mpfh.Pid.SetPoint
                        if mpfh.Pid.SetPoint > 0:
                            feedback_fr =mpfh.Abs_Encoder_fr_id2_oct
                            print "Feedback---fr",feedback_fr
                        # if i>1:
                        #     mpfh.Pid.SetPoint = mpfh.fl_abs_encode
                        #     print "------fl_abs_encode",mpfh.Pid.SetPoint,mpfh.fl_abs_encode
                        if len(output_fr)>3:
                            velocity_control=mpfh.Pid.Kp*(output_fr[count_num_fr]-output_fr[count_num_fr-1])+mpfh.Pid.Ki*output_fr[count_num_fr]+mpfh.Pid.Kd*(output_fr[count_num_fr]-2*output_fr[count_num_fr-1]+output_fr[count_num_fr-2])
                            out_vel=(velocity_control*60*50)/1024
                            print "out_vel-----fr",out_vel
                            mpfh.MobileControl.Send_Velocity_Driver(int(out_vel),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                            print "count num",count_num_fr
                            # time.sleep(0.01)
                        feedback_list_fr.append(feedback_fr)
                        setpoint_list_fr.append(mpfh.Pid.SetPoint)
                        time_list_fr.append(i)
                        if abs(mpfh.Pid.Error)<=1:
                            mpfh.MobileControl.Send_Velocity_Driver(0,'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn1'])
                            flg_fr=0
                            # count_num_fr=0
                            output_fr=[]
                        count_num_fr+=1
                    if flg_rl == 'rl':
                        mpfh.Pid.SetPoint = mpfh.rl_abs_encode
                        mpfh.Pid.update(feedback_rl)
                        output = mpfh.Pid.output
                        output_rl.append(output)
                        print "output---rl",output
                        
                        print "------rl_abs_encode",mpfh.Pid.SetPoint
                        if mpfh.Pid.SetPoint > 0:
                            feedback_rl =mpfh.Abs_Encoder_rl_id3_oct
                            print "Feedback-----rl",feedback_rl
                        # if i>1:
                        #     mpfh.Pid.SetPoint = mpfh.fl_abs_encode
                        #     print "------fl_abs_encode",mpfh.Pid.SetPoint,mpfh.fl_abs_encode
                        if len(output_rl)>3:
                            velocity_control=mpfh.Pid.Kp*(output_rl[count_num_rl]-output_rl[count_num_rl-1])+mpfh.Pid.Ki*output_rl[count_num_rl]+mpfh.Pid.Kd*(output_rl[count_num_rl]-2*output_rl[count_num_rl-1]+output_rl[count_num_rl-2])
                            out_vel=(velocity_control*60*50)/1024
                            print "out_vel-----rl",out_vel,len(output_rl)
                            mpfh.MobileControl.Send_Velocity_Driver(int(out_vel),'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                            print "count num rl",count_num_rl
                            # time.sleep(0.01)
                        feedback_list_rl.append(feedback_rl)
                        setpoint_list_rl.append(mpfh.Pid.SetPoint)
                        time_list_rl.append(i)
                        if abs(mpfh.Pid.Error)<=1:
                            mpfh.MobileControl.Send_Velocity_Driver(0,'left',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                            flg_rl=0
                            # count_num_rl=0
                            output_rl=[]
                        count_num_rl+=1
                    if flg_rr == 'rr':
                        mpfh.Pid.SetPoint = mpfh.rr_abs_encode
                        mpfh.Pid.update(feedback_rr)
                        output = mpfh.Pid.output
                        output_rr.append(output)
                        print "output",output
                        
                        print "------rr_abs_encode",mpfh.Pid.SetPoint
                        if mpfh.Pid.SetPoint > 0:
                            feedback_rr =mpfh.Abs_Encoder_rr_id4_oct
                            print "Feedback-----rr",feedback_rr

                        if len(output_rr)>3:
                            velocity_control=mpfh.Pid.Kp*(output_rr[count_num_rr]-output_rr[count_num_rr-1])+mpfh.Pid.Ki*output_rr[count_num_rr]+mpfh.Pid.Kd*(output_rr[count_num_rr]-2*output_rr[count_num_rr-1]+output_rr[count_num_rr-2])
                            out_vel=(velocity_control*60*50)/1024
                            print "out_vel-----fl",out_vel
                            mpfh.MobileControl.Send_Velocity_Driver(int(out_vel),'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                            print "count num",count_num_rr
                            # time.sleep(0.01)
                        feedback_list_rr.append(feedback_rr)
                        setpoint_list_rr.append(mpfh.Pid.SetPoint)
                        time_list_rr.append(i)
                        if abs(mpfh.Pid.Error)<=1:
                            mpfh.MobileControl.Send_Velocity_Driver(0,'right',mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel']['chn2'])
                            flg_rr=0
                            # count_num_rr=0
                            output_rr=[]
                        count_num_rr+=1
                                                                      
                else:
                    mpfh.New_Read_Encoder_data_From_ABS_Encoder(recevenum)
                    print "read data"
                    time.sleep(0.1)
        # print(time_list_fl) 
        # print(feedback_list_fl)
        # print(setpoint_list_fl)
        mpfh.Pid_control_Performance(feedback_list_fl,setpoint_list_fl,time_list_fl,0,END) 
        mpfh.Pid_control_Performance(feedback_list_fr,setpoint_list_fr,time_list_fr,1,END) 
        mpfh.Pid_control_Performance(feedback_list_rl,setpoint_list_rl,time_list_rl,2,END) 
        mpfh.Pid_control_Performance(feedback_list_rr,setpoint_list_rr,time_list_rr,3,END) 
        # print(time_list_fl) 
        # print(feedback_list_fl)
        # print(setpoint_list_fl)
        # time_sm = np.array(time_list_fl)
        # time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)
        # feedback_smooth = spline(time_list_fl, feedback_list_fl, time_smooth)
        # plt.figure(0)
        # plt.plot(time_smooth, feedback_smooth)
        # plt.plot(time_list_fl, setpoint_list_fl)
        # plt.xlim((0, END))
        # plt.ylim((min(feedback_list_fl)-0.5, max(feedback_list_fl)+0.5))
        # plt.xlabel('time (s)')
        # plt.ylabel('PID (PV)')
        # plt.title('TEST PID')

        # plt.ylim((1-50000.5, 1+50000.5))
        # # plt.ylim((1-0.5, 1+0.5))
        # plt.grid(True)
        # plt.show()            
    mpfh.MobileControl.CanAnalysis.Can_VCICloseDevice()
if __name__=="__main__":
    main()