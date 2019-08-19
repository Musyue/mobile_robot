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
class MobilePlatformDriver():
    def __init__(self,):
        self.CanAnalysis=CanAnalysisDriver()
        self.CanAnalysis.Init_Can_All() #初始化can分析仪，注意这里没有关闭分析仪
        self.MobileDriver_Command=MobileDriverCommands()
        self.logger=LoggerSetClass(1)
        self.Datalen=8
    def Enable_Motor_controller(self,IDD,Frames_Length):
        
        enbalecomd1=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],Frames_Length,IDD,self.Datalen,self.MobileDriver_Command.ENABLE_COMMAND_1)
        time.sleep(0.3)
        if enbalecomd1:
            ret,kk=self.CanAnalysis.Can_Receive(0,1)
            if ret:
                if list(kk.Data)[0]!=0 and list(kk.Data)[0]!=127:
                    print('driver feedback data:--->',list(kk.Data))
                    print(kk.DataLen)
            enbalecomd2=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],Frames_Length,IDD,self.Datalen,self.MobileDriver_Command.ENABLE_COMMAND_2)
            time.sleep(0.3)
            ret,kk=self.CanAnalysis.Can_Receive(0,1)
            if ret:
                if list(kk.Data)[0]!=0 and list(kk.Data)[0]!=127:
                    print('driver feedback data:--->',list(kk.Data))
                    print(kk.DataLen)
            if enbalecomd2:
                enbalecomd3=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],Frames_Length,IDD,self.Datalen,self.MobileDriver_Command.ENABLE_COMMAND_3)
                time.sleep(0.3)
                ret,kk=self.CanAnalysis.Can_Receive(0,1)
                if ret:
                    if list(kk.Data)[0]!=0 and list(kk.Data)[0]!=127:
                        print('driver feedback data:--->',list(kk.Data))
                        print(kk.DataLen)
        else:
            self.logger.loggererror("Please check your controller or can analysis!!!!")
        return True
    def Enable_Steering_Controller(self):
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel']['chn1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.1)
        if Mc03:
            self.logger.loggerinfo("Enable Front Steering Motor Controller","GREEN")
        Mc04=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel']['chn2'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.1)
        if Mc04:
            self.logger.loggerinfo("Enable Rear Steering Motor Controller","GREEN")
    def Enable_Walking_Controller(self):
        Mc01=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel']['chn1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.1)
        if Mc01:
            self.logger.loggerinfo("Enable Front Walking Motor Controller","GREEN")
        Mc02=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel']['chn2'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.1)
        if Mc02:
            self.logger.loggerinfo("Enable Rear Walking Motor Controller","GREEN")
    def Enable_Motor_Controller_All(self):
        Mc01=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel']['chn1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.1)
        if Mc01:
            self.logger.loggerinfo("Enable Front Walking Motor Controller","GREEN")
        Mc02=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel']['chn2'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.1)
        if Mc02:
            self.logger.loggerinfo("Enable Rear Walking Motor Controller","GREEN")
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel']['chn1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.1)
        if Mc03:
            self.logger.loggerinfo("Enable Front Steering Motor Controller","GREEN")
        Mc04=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel']['chn2'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.1)
        if Mc04:
            self.logger.loggerinfo("Enable Rear Steering Motor Controller","GREEN")
    def Disable_Motor_controller(self,IDD,Frames_Length):
        
        disablecomd=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],Frames_Length,IDD,self.Datalen,self.MobileDriver_Command.DISENABLE_COMMAND)
        time.sleep(0.1)
        if disablecomd:
            strinfo="motor controller ID:"+str(IDD)+'is Disabled!!!'
            self.logger.loggerinfo(strinfo)
        return True
    def Disable_ALL_Motor_Controller(self):
        
        Mc01=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel']['chn1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.1)
        if Mc01:
            self.logger.loggerinfo("Disable Front Walking Motor Controller","GREEN")
        Mc02=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel']['chn2'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.1)
        if Mc02:
            self.logger.loggerinfo("Disable Rear Walking Motor Controller","GREEN")
        Mc03=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel']['chn1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.1)
        if Mc03:
            self.logger.loggerinfo("Disable Front Steering Motor Controller","GREEN")
        Mc04=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel']['chn2'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.1)
        if Mc04:
            self.logger.loggerinfo("Disable Rear Steering Motor Controller","GREEN")
    def Opreation_Controller_Mode(self,IDD,Mode_Choice):
        """
        Mode_Choice: Three Mode :Velocity,Position,Electric
        """
        
        mod_cmd=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],self.CanAnalysis.yamlDic['Frames_Length'],IDD,self.Datalen,Mode_Choice)
        time.sleep(0.1)
        if mod_cmd:
            infostr='Mode:'+str(Mode_Choice)+'Set Ok!!!'
            self.logger.loggerinfo(infostr)
            return True
        else:
            self.logger.loggererror("Please Check your controller or USB CAN Analysis!!!!")
            return False
    
    def Send_Control_Command(self,IDD,Commands):
        
        mod_cmd=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],self.CanAnalysis.yamlDic['Frames_Length'],IDD,self.Datalen,Commands)
        time.sleep(0.1)
        if mod_cmd:
            infostr='Command:'+str(Commands)+'send Ok!!!'
            self.logger.loggerinfo(infostr)
            return True
        else:
            self.logger.loggererror("Please Check your controller or USB CAN Analysis!!!!")
            return False
    def Save_Parameter(self,flag):
        if flag=="steer":
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileDriver_Command.SAVE_PARAMETERS)
            time.sleep(0.1)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileDriver_Command.SAVE_PARAMETERS)
            time.sleep(0.1)
        elif flag=="walking":
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel']['chn1'],self.MobileDriver_Command.SAVE_PARAMETERS)
            time.sleep(0.1)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel']['chn2'],self.MobileDriver_Command.SAVE_PARAMETERS)
            time.sleep(0.1)
        else:
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileDriver_Command.SAVE_PARAMETERS)
            time.sleep(0.1)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileDriver_Command.SAVE_PARAMETERS)
            time.sleep(0.1)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel']['chn1'],self.MobileDriver_Command.SAVE_PARAMETERS)
            time.sleep(0.1)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel']['chn2'],self.MobileDriver_Command.SAVE_PARAMETERS)
            time.sleep(0.1)

    def Send_Zero_Velocity_to_All_Motor(self):
        self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileDriver_Command.BASIC_LEFT_TARGET_VELOCITY_COMMAND)
        time.sleep(0.1)
        self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileDriver_Command.BASIC_LEFT_TARGET_VELOCITY_COMMAND)
        time.sleep(0.1)
        self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel']['chn1'],self.MobileDriver_Command.BASIC_LEFT_TARGET_VELOCITY_COMMAND)
        time.sleep(0.1)
        self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel']['chn2'],self.MobileDriver_Command.BASIC_LEFT_TARGET_VELOCITY_COMMAND)
        time.sleep(0.1)
        self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileDriver_Command.BASIC_RIGHT_TARGET_VELOCITY_COMMAND)
        time.sleep(0.1)
        self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileDriver_Command.BASIC_RIGHT_TARGET_VELOCITY_COMMAND)
        time.sleep(0.1)
        self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel']['chn1'],self.MobileDriver_Command.BASIC_RIGHT_TARGET_VELOCITY_COMMAND)
        time.sleep(0.1)
        self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel']['chn2'],self.MobileDriver_Command.BASIC_RIGHT_TARGET_VELOCITY_COMMAND)
        time.sleep(0.1)
    def Caculate_Velocity_Command(self,VelocityData,Left_Right,Driver_Id):
        if VelocityData<0:
        velocity=hex(VelocityData & 0xFFFFFFFF)
    def Caculate_Position_Command(self,PositionData,Left_Right,Driver_Id):
        pass
def main():
    mpd=MobilePlatformDriver()
    count=1
    while count:
        mpd.Send_Control_Command(mpd.CanAnalysis.yamlDic['steering_channel']['chn2'],mpd.MobileDriver_Command.BASIC_LEFT_TARGET_VELOCITY_COMMAND)
        # if self.Opreation_Controller_Mode(self.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileDriver_Command.SET_MODE_VELOCITY):
            # if  self.Enable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel']['chn2'],1):
            #     self.logger.loggerinfo("Enable")
            #     if self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileDriver_Command.BASIC_LEFT_TEST_VELOCITY_COMMAND):
            #         self.logger.loggerinfo("OK VELOCITY","LIGHT_RED")
            #         time.sleep(2)
        count-=1
    mpd.CanAnalysis.Can_VCICloseDevice()
if __name__=="__main__":
    main()