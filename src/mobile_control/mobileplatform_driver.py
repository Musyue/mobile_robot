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
        time.sleep(0.01)
        if enbalecomd1:
            ret,kk=self.CanAnalysis.Can_Receive(0,1)
            if ret:
                if list(kk.Data)[0]!=0 and list(kk.Data)[0]!=127:
                    print('driver feedback data:--->',list(kk.Data))
                    print(kk.DataLen)
            enbalecomd2=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],Frames_Length,IDD,self.Datalen,self.MobileDriver_Command.ENABLE_COMMAND_2)
            time.sleep(0.01)
            ret,kk=self.CanAnalysis.Can_Receive(0,1)
            if ret:
                if list(kk.Data)[0]!=0 and list(kk.Data)[0]!=127:
                    print('driver feedback data:--->',list(kk.Data))
                    print(kk.DataLen)
            if enbalecomd2:
                enbalecomd3=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],Frames_Length,IDD,self.Datalen,self.MobileDriver_Command.ENABLE_COMMAND_3)
                time.sleep(0.01)
                ret,kk=self.CanAnalysis.Can_Receive(0,1)
                if ret:
                    if list(kk.Data)[0]!=0 and list(kk.Data)[0]!=127:
                        print('driver feedback data:--->',list(kk.Data))
                        # print(kk.DataLen)
        else:
            self.logger.loggererror("Please check your controller or can analysis!!!!")
        return True
    def Enable_Steering_Controller(self):
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel']['chn1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.01)
        if Mc03:
            self.logger.loggerinfo("Enable Front Steering Motor Controller","GREEN")
        Mc04=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel']['chn2'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.01)
        if Mc04:
            self.logger.loggerinfo("Enable Rear Steering Motor Controller","GREEN")
    def Enable_Walking_Controller(self):
        Mc01=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel']['chn1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.01)
        if Mc01:
            self.logger.loggerinfo("Enable Front Walking Motor Controller","GREEN")
        Mc02=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel']['chn2'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.01)
        if Mc02:
            self.logger.loggerinfo("Enable Rear Walking Motor Controller","GREEN")
    def Enable_Motor_Controller_All(self):
        Mc01=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel']['chn1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.01)
        if Mc01:
            self.logger.loggerinfo("Enable Front Walking Motor Controller","GREEN")
        Mc02=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel']['chn2'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.01)
        if Mc02:
            self.logger.loggerinfo("Enable Rear Walking Motor Controller","GREEN")
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel']['chn1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.01)
        if Mc03:
            self.logger.loggerinfo("Enable Front Steering Motor Controller","GREEN")
        Mc04=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel']['chn2'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.01)
        if Mc04:
            self.logger.loggerinfo("Enable Rear Steering Motor Controller","GREEN")
    def Disable_Motor_controller(self,IDD,Frames_Length):
        
        disablecomd=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],Frames_Length,IDD,self.Datalen,self.MobileDriver_Command.DISENABLE_COMMAND)
        time.sleep(0.01)
        if disablecomd:
            strinfo="motor controller ID:"+str(IDD)+'is Disabled!!!'
            self.logger.loggerinfo(strinfo)
        return True
    def Disable_ALL_Motor_Controller(self):
        
        Mc01=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel']['chn1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.01)
        if Mc01:
            self.logger.loggerinfo("Disable Front Walking Motor Controller","GREEN")
        Mc02=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel']['chn2'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.01)
        if Mc02:
            self.logger.loggerinfo("Disable Rear Walking Motor Controller","GREEN")
        Mc03=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel']['chn1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.01)
        if Mc03:
            self.logger.loggerinfo("Disable Front Steering Motor Controller","GREEN")
        Mc04=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel']['chn2'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.01)
        if Mc04:
            self.logger.loggerinfo("Disable Rear Steering Motor Controller","GREEN")
    def Opreation_Controller_Mode(self,IDD,Mode_Choice):
        """
        Mode_Choice: Three Mode :Velocity,Position,Electric
        """
        
        mod_cmd=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],self.CanAnalysis.yamlDic['Frames_Length'],IDD,self.Datalen,Mode_Choice)
        time.sleep(0.01)
        if mod_cmd:
            infostr='Mode:'+str(Mode_Choice)+'Set Ok!!!'
            self.logger.loggerinfo(infostr)
            return True
        else:
            self.logger.loggererror("Please Check your controller or USB CAN Analysis!!!!")
            return False
    
    def Send_Control_Command(self,IDD,Commands):
        
        mod_cmd=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],self.CanAnalysis.yamlDic['Frames_Length'],IDD,self.Datalen,Commands)
        time.sleep(0.01)
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
            time.sleep(0.01)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileDriver_Command.SAVE_PARAMETERS)
            time.sleep(0.01)
        elif flag=="walking":
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel']['chn1'],self.MobileDriver_Command.SAVE_PARAMETERS)
            time.sleep(0.01)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel']['chn2'],self.MobileDriver_Command.SAVE_PARAMETERS)
            time.sleep(0.01)
        else:
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileDriver_Command.SAVE_PARAMETERS)
            time.sleep(0.01)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileDriver_Command.SAVE_PARAMETERS)
            time.sleep(0.01)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel']['chn1'],self.MobileDriver_Command.SAVE_PARAMETERS)
            time.sleep(0.01)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel']['chn2'],self.MobileDriver_Command.SAVE_PARAMETERS)
            time.sleep(0.01)

    def Send_Zero_Velocity_to_All_Motor(self):
        self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileDriver_Command.BASIC_LEFT_TARGET_VELOCITY_COMMAND)
        time.sleep(0.01)
        self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileDriver_Command.BASIC_LEFT_TARGET_VELOCITY_COMMAND)
        time.sleep(0.01)
        self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel']['chn1'],self.MobileDriver_Command.BASIC_LEFT_TARGET_VELOCITY_COMMAND)
        time.sleep(0.01)
        self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel']['chn2'],self.MobileDriver_Command.BASIC_LEFT_TARGET_VELOCITY_COMMAND)
        time.sleep(0.01)
        self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel']['chn2'],self.MobileDriver_Command.BASIC_RIGHT_TARGET_VELOCITY_COMMAND)
        time.sleep(0.01)
        self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel']['chn1'],self.MobileDriver_Command.BASIC_RIGHT_TARGET_VELOCITY_COMMAND)
        time.sleep(0.01)
        self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel']['chn1'],self.MobileDriver_Command.BASIC_RIGHT_TARGET_VELOCITY_COMMAND)
        time.sleep(0.01)
        self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel']['chn2'],self.MobileDriver_Command.BASIC_RIGHT_TARGET_VELOCITY_COMMAND)
        time.sleep(0.01)
    def Send_Velocity_Driver(self,VelocityData,Left_Right,Driver_Id):
        command=self.Caculate_Velocity_Command(int(VelocityData),Left_Right)
        self.Send_Control_Command(Driver_Id,command)
        time.sleep(0.01)
    def Caculate_Velocity_Command(self,VelocityData,Left_Right):
        """
        LEFT:(0x23, 0xff, 0x60, 0x00, 0x58, 0x02, 0x00, 0x00)
        RIGHT:(0x23, 0xff, 0x68, 0x00, 0xe8, 0x03, 0x00, 0x00)
        """
        LEFT=self.MobileDriver_Command.BASIC_LEFT_TARGET_VELOCITY_COMMAND#(0x23, 0xff, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00)
        RIGHT=self.MobileDriver_Command.BASIC_RIGHT_TARGET_VELOCITY_COMMAND#(0x23, 0xff, 0x68, 0x00, 0x00, 0x00, 0x00, 0x00)
        NEWLEFT=()
        NEWRIGHT=()
        if VelocityData<0:
            if Left_Right=='left' or Left_Right=='LEFT':
                velocity=hex(VelocityData & 0xFFFFFFFF)
                newvelocity=[velocity[i:i+2] for i in range(0,len(velocity), 2)]
                NEWLEFT=LEFT[:4]+(int(newvelocity[4],16),int(newvelocity[3],16),int(newvelocity[2],16),int(newvelocity[1],16))
                print newvelocity,NEWLEFT
                return NEWLEFT
            elif Left_Right =="right" or Left_Right=="RIGHT":
                velocity=hex(VelocityData & 0xFFFFFFFF)
                newvelocity=[velocity[i:i+2] for i in range(0,len(velocity), 2)]
                NEWRIGHT=RIGHT[:4]+(int(newvelocity[4],16),int(newvelocity[3],16),int(newvelocity[2],16),int(newvelocity[1],16))
                print newvelocity,NEWRIGHT
                return NEWRIGHT
            else:
                print("Please input LEFT/RIGHT or left/right  -----")
        elif VelocityData>0:
            velocity=hex(VelocityData).replace('0x','')
            # print velocity
            if len(velocity)==1:
                newvelocity=velocity.zfill(len(velocity)+3)
                tempnewvelocity=[newvelocity[i:i+2] for i in range(0,len(newvelocity), 2)]
                # print tempnewvelocity
            elif len(velocity)==2:
                newvelocity=velocity.zfill(len(velocity)+2)
                tempnewvelocity=[newvelocity[i:i+2] for i in range(0,len(newvelocity), 2)]
                # print tempnewvelocity
            elif len(velocity)==3:
                newvelocity=velocity.zfill(len(velocity)+1)
                tempnewvelocity=[newvelocity[i:i+2] for i in range(0,len(newvelocity), 2)]
                # print tempnewvelocity
            else:
                tempnewvelocity=[velocity[i:i+2] for i in range(0,len(velocity), 2)]
                # print tempnewvelocity
            if Left_Right=='left' or Left_Right=='LEFT':
                NEWLEFT=LEFT[:4]+(int(tempnewvelocity[1],16),int(tempnewvelocity[0],16))+LEFT[6:]
                # print NEWLEFT
                return NEWLEFT
            elif Left_Right =="right" or Left_Right=="RIGHT":
                NEWRIGHT=RIGHT[:4]+(int(tempnewvelocity[1],16),int(tempnewvelocity[0],16))+RIGHT[6:]
                # print NEWRIGHT
                return NEWRIGHT
            else:
                print("Please input LEFT/RIGHT or left/right  -----")
        else:
            if Left_Right=='left' or Left_Right=='LEFT':
                return LEFT
            elif Left_Right =="right" or Left_Right=="RIGHT":
                return RIGHT
            else:
                print("Please input LEFT/RIGHT or left/right  -----")

    def Caculate_Position_Command(self,PositionData,Left_Right,Driver_Id):
        pass
def main():
    mpd=MobilePlatformDriver()
    count=1
    flag=1
    while count:
        if flag==1:
            mpd.Send_Velocity_Driver(0,'left',mpd.CanAnalysis.yamlDic['steering_channel']['chn2'])
            mpd.Send_Velocity_Driver(0,'right',mpd.CanAnalysis.yamlDic['steering_channel']['chn2'])
            mpd.Send_Velocity_Driver(0,'left',mpd.CanAnalysis.yamlDic['steering_channel']['chn1'])
            # mpd.Send_Control_Command(mpd.CanAnalysis.yamlDic['steering_channel']['chn2'],(int('23',16),int('ff',16),int('60',16),int('00',16),int('c8',16),int('00',16),int('00',16),int('00',16)))
            mpd.Send_Velocity_Driver(0,'right',mpd.CanAnalysis.yamlDic['steering_channel']['chn1'])
        else:
            mpd.Send_Velocity_Driver(1000,'left',mpd.CanAnalysis.yamlDic['steering_channel']['chn2'])
            mpd.Send_Velocity_Driver(-1000,'right',mpd.CanAnalysis.yamlDic['steering_channel']['chn2'])
            mpd.Send_Velocity_Driver(1000,'left',mpd.CanAnalysis.yamlDic['steering_channel']['chn1'])
            #mpd.Send_Control_Command(mpd.CanAnalysis.yamlDic['steering_channel']['chn2'],(int('23',16),int('ff',16),int('60',16),int('00',16),int('c8',16),int('00',16),int('00',16),int('00',16)))
            mpd.Send_Velocity_Driver(1000,'right',mpd.CanAnalysis.yamlDic['steering_channel']['chn1'])#0x23, 0xff, 0x60, 0x00, 0xc8, 0x00, 0x00, 0x00)
        # mpd.Send_Control_Command(mpd.CanAnalysis.yamlDic['steering_channel']['chn2'],mpd.MobileDriver_Command.BASIC_LEFT_TARGET_VELOCITY_COMMAND)
        # if mpd.Opreation_Controller_Mode(mpd.CanAnalysis.yamlDic['steering_channel']['chn2'],mpd.MobileDriver_Command.SET_MODE_VELOCITY):
        #     if  mpd.Enable_Motor_controller(mpd.CanAnalysis.yamlDic['steering_channel']['chn2'],1):
        #         mpd.logger.loggererror("Enable")
        #         if mpd.Send_Control_Command(mpd.CanAnalysis.yamlDic['steering_channel']['chn2'],mpd.MobileDriver_Command.BASIC_LEFT_TEST_600_VELOCITY_COMMAND):
        #             mpd.logger.loggerinfo("OK VELOCITY","LIGHT_RED")
        #             time.sleep(2)
        count-=1
    mpd.CanAnalysis.Can_VCICloseDevice()
if __name__=="__main__":
    main()