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
                    print('abs encoder 1',self.List_to_HEXList(list(kk[i].Data)))
                    # print(kk[i].DataLen)
                    self.Abs_Encoder_fl_id1.append(self.List_to_HEXList(list(kk[i].Data)))
                    self.Abs_Encoder_fl_id1_oct=self.HEX_String_List_To_Oct(list(kk[i].Data))
                    self.MobileControl.logger.loggerinfo(self.HEX_String_List_To_Oct(list(kk[i].Data)))
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and list(kk[i].Data)[1]==2:
                    print('abs encoder 2',self.List_to_HEXList(list(kk[i].Data)))
                    # print(kk[i].DataLen)
                    self.Abs_Encoder_fr_id2.append(self.List_to_HEXList(list(kk[i].Data)))
                    self.Abs_Encoder_fr_id2_oct=self.HEX_String_List_To_Oct(list(kk[i].Data))
                    self.MobileControl.logger.loggerinfo(self.HEX_String_List_To_Oct(list(kk[i].Data)))
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127 and list(kk[i].Data)[1]==3:
                    print('abs Encoder 3',self.List_to_HEXList(list(kk[i].Data)))
                    # print(kk[i].DataLen)
                    self.Abs_Encoder_rl_id3.append(self.List_to_HEXList(list(kk[i].Data)))
                    self.MobileControl.logger.loggerinfo(self.HEX_String_List_To_Oct(list(kk[i].Data)))
                    self.Abs_Encoder_rl_id3_oct=self.HEX_String_List_To_Oct(list(kk[i].Data))
                if list(kk[i].Data)[0]!=0 and list(kk[i].Data)[0]!=127and list(kk[i].Data)[1]==4:
                    print('abs Encoder 4',self.List_to_HEXList(list(kk[i].Data)))
                    # print(kk[i].DataLen)
                    self.Abs_Encoder_rr_id4.append(self.List_to_HEXList(list(kk[i].Data)))
                    self.Abs_Encoder_rr_id4_oct=self.HEX_String_List_To_Oct(list(kk[i].Data))
                    self.MobileControl.logger.loggerinfo(self.HEX_String_List_To_Oct(list(kk[i].Data)))             

    def Read_Encoder_data_From_ABS_Encoder(self,):
        # self.MobileControl.CanAnalysis.Can_ClearBuffer(0)
        transmit_status_1=self.MobileControl.CanAnalysis.Can_Transmit(0,1,1,8,self.MobileControl.MobileDriver_Command.REQUEST_ENCODER_1)
        if transmit_status_1:
            # time.sleep(0.1)

            ret,kk=self.MobileControl.CanAnalysis.Can_Receive(0,1)
            if ret:
                if list(kk.Data)[0]!=0 and list(kk.Data)[0]!=127 and list(kk.Data)[1]==1:
                    print('abs encoder 1',self.List_to_HEXList(list(kk.Data)))
                    print(kk.DataLen)
                    self.Abs_Encoder_fl_id1.append(self.List_to_HEXList(list(kk.Data)))
                    self.Abs_Encoder_fl_id1_oct=self.HEX_String_List_To_Oct(self.List_to_HEXList(list(kk.Data)))
                    self.MobileControl.logger.loggerinfo(self.HEX_String_List_To_Oct(self.List_to_HEXList(list(kk.Data))))
        else:
            pass
        transmit_status_2=self.MobileControl.CanAnalysis.Can_Transmit(0,1,2,8,self.MobileControl.MobileDriver_Command.REQUEST_ENCODER_2)
        if transmit_status_2:
            # time.sleep(0.1)
            ret,kk=self.MobileControl.CanAnalysis.Can_Receive(0,1)
            if ret:
                if list(kk.Data)[0]!=0 and list(kk.Data)[0]!=127 and list(kk.Data)[1]==2:
                    print('abs encoder 2',self.List_to_HEXList(list(kk.Data)))
                    print(kk.DataLen)
                    self.Abs_Encoder_fr_id2.append(self.List_to_HEXList(list(kk.Data)))
                    self.Abs_Encoder_fr_id2_oct=self.HEX_String_List_To_Oct(self.List_to_HEXList(list(kk.Data)))
                    self.MobileControl.logger.loggerinfo(self.HEX_String_List_To_Oct(self.List_to_HEXList(list(kk.Data))))
        else:
            pass
        transmit_status_3=self.MobileControl.CanAnalysis.Can_Transmit(0,1,3,8,self.MobileControl.MobileDriver_Command.REQUEST_ENCODER_3)
        if transmit_status_3:
            # time.sleep(0.1)
            ret,kk=self.MobileControl.CanAnalysis.Can_Receive(0,1)
            if ret:
                if list(kk.Data)[0]!=0 and list(kk.Data)[0]!=127 and list(kk.Data)[1]==3:
                    print('abs Encoder 3',self.List_to_HEXList(list(kk.Data)))
                    print(kk.DataLen)
                    self.Abs_Encoder_rl_id3.append(self.List_to_HEXList(list(kk.Data)))
                    self.MobileControl.logger.loggerinfo(self.HEX_String_List_To_Oct(self.List_to_HEXList(list(kk.Data))))
                    self.Abs_Encoder_rl_id3_oct=self.HEX_String_List_To_Oct(self.List_to_HEXList(list(kk.Data)))

        else:
            pass
        transmit_status_4=self.MobileControl.CanAnalysis.Can_Transmit(0,1,4,8,self.MobileControl.MobileDriver_Command.REQUEST_ENCODER_4)
        if transmit_status_4:
            # time.sleep(0.1)
            ret,kk=self.MobileControl.CanAnalysis.Can_Receive(0,1)
            if ret:
                if list(kk.Data)[0]!=0 and list(kk.Data)[0]!=127and list(kk.Data)[1]==4:
                    print('abs Encoder 4',self.List_to_HEXList(list(kk.Data)))
                    print(kk.DataLen)
                    self.Abs_Encoder_rr_id4.append(self.List_to_HEXList(list(kk.Data)))
                    self.Abs_Encoder_rr_id4_oct=self.HEX_String_List_To_Oct(self.List_to_HEXList(list(kk.Data)))
                    self.MobileControl.logger.loggerinfo(self.HEX_String_List_To_Oct(self.List_to_HEXList(list(kk.Data))))
        else:
            pass
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
    def Homing(self):
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
def main():
    mpfh=MobilePlatFormHoming()
    mpfh.Init_mobile_platform()
    count=1
    flag=0
    recevenum=0
    flag_1=1
    if flag:
        mpfh.Stop_Four_Steer_Motor('all')
    else:
        # mpfh.Run_Four_Steer_Motor('all')
        # time.sleep(0.4)
        while count:
            recevenum=mpfh.MobileControl.CanAnalysis.Can_GetReceiveNum(0)
            print("RECENUM------->",recevenum)

            if recevenum!=None:
                if flag_1==1 and mpfh.Abs_Encoder_fr_id2_oct!=0 and mpfh.Abs_Encoder_fl_id1_oct!=0 and mpfh.Abs_Encoder_rr_id4_oct!=0 and mpfh.Abs_Encoder_rl_id3_oct!=0:
                    mpfh.Run_Four_Steer_Motor_New()
                    flag_1=0
                else:
                    mpfh.New_Read_Encoder_data_From_ABS_Encoder(recevenum)
                    mpfh.Homing()
                    count+=1
                    time.sleep(0.1)
                    print(mpfh.home_ok_flag)
                    if mpfh.home_ok_flag.has_key('steer_chn1_left') and mpfh.home_ok_flag.has_key('steer_chn1_right') and mpfh.home_ok_flag.has_key('steer_chn2_left') and mpfh.home_ok_flag.has_key('steer_chn2_right'):
                        if mpfh.home_ok_flag['steer_chn1_left']==1 and mpfh.home_ok_flag['steer_chn1_right']==1 and mpfh.home_ok_flag['steer_chn2_left']==1 and mpfh.home_ok_flag['steer_chn2_right']==1:
                            mpfh.MobileControl.logger.loggerinfo("------------------!!!!!!Homing is over !!!!!!-----------","LIGHT_RED")
                            break
                
    mpfh.MobileControl.CanAnalysis.Can_VCICloseDevice()
if __name__=="__main__":
    main()