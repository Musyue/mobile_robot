#! /usr/bin/env python
# coding=utf-8
import sys
sys.path.append("..")
from ctypes import *
import yaml
import os
from math import *
from logger_config.logger_set import *
from config.command import *
from can_analysis.can_analysis_driver import *
import time
class MobilePlatformDriver():
    def __init__(self,):
        self.CanAnalysis=CanAnalysisDriver()
        self.CanAnalysis.Init_Can_All()
        self.MobileDriver_Command=MobileDriverCommands()
        self.logger=LoggerSetClass(1)
        self.Datalen=8
    def Init_can(self):
        self.CanAnalysis.Init_Can_All() #初始化can分析仪，注意这里没有关闭分析仪
    def Open_driver_can_Node(self,IDD,Frames_Length):
        enbalecomd1=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],Frames_Length,0,self.Datalen,self.MobileDriver_Command.OPEN_CAN_NODE_DRIVER)
        time.sleep(0.0015)
    def Enable_Motor_controller(self,IDD,Frames_Length,flag):
        """
        flag=1,velocity
        flag=0,postion
        """
        if flag:
            enbalecomd1=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],Frames_Length,IDD,self.Datalen,self.MobileDriver_Command.ENABLE_DRIVER_SET_VELOCITY)
            time.sleep(0.0015)
        else:
            enbalecomd1=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],Frames_Length,IDD,self.Datalen,self.MobileDriver_Command.ENABLE_DRIVER_SET_POSITION)
            time.sleep(0.0015)
        
    def Enable_Steering_Controller(self,flag):
        self.logger.loggerinfo("Enable Steering Motor Controller","GREEN")
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],flag)
        time.sleep(0.001)
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],flag)
        time.sleep(0.001)
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],flag)
        time.sleep(0.001)
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],flag)
        time.sleep(0.001)

    def Enable_Walking_Controller(self,flag):
        self.logger.loggerinfo("Enable Walking Motor Controller","GREEN")
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],flag)
        time.sleep(0.001)
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],flag)
        time.sleep(0.001)
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],flag)
        time.sleep(0.001)
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],flag)
        time.sleep(0.001)
    def Enable_Motor_Controller_All(self):
        self.logger.loggerinfo("Enable Steering and Walking Motor Controller","GREEN")
        self.Enable_Steering_Controller(1)
        self.Enable_Walking_Controller(1)
    def Disable_Motor_controller(self,IDD,Frames_Length):
        
        disablecomd=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],Frames_Length,IDD,self.Datalen,self.MobileDriver_Command.DISABLE_DRIVER)
        time.sleep(0.001)
        if disablecomd:
            strinfo="motor controller ID:"+str(IDD)+'is Disabled!!!'
            self.logger.loggerinfo(strinfo)
        return True
    def Disable_ALL_Motor_Controller(self):
        
        Mc01=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.001)
        if Mc01:
            self.logger.loggerinfo("Disable Front left Walking Motor Controller","GREEN")
        Mc02=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.001)
        if Mc02:
            self.logger.loggerinfo("Disable Front right Walking Motor Controller","GREEN")
        Mc03=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.001)
        if Mc03:
            self.logger.loggerinfo("Disable Rear Left Walking Motor Controller","GREEN")
        Mc04=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.001)
        if Mc04:
            self.logger.loggerinfo("Disable Rear Right Walking Motor Controller","GREEN")
        Mc05=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.001)
        if Mc05:
            self.logger.loggerinfo("Disable Front left Walking Motor Controller","GREEN")
        Mc06=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.001)
        if Mc06:
            self.logger.loggerinfo("Disable Front right Walking Motor Controller","GREEN")
        Mc07=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.001)
        if Mc07:
            self.logger.loggerinfo("Disable Rear Left Walking Motor Controller","GREEN")
        Mc08=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.001)
        if Mc08:
            self.logger.loggerinfo("Disable Rear Right Walking Motor Controller","GREEN")
    def Opreation_Controller_Mode(self,IDD,Mode_Choice):
        pass
        # """
        # Mode_Choice: Three Mode :Velocity,Position,Electric
        # """
        
        # mod_cmd=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],self.CanAnalysis.yamlDic['Frames_Length'],IDD,self.Datalen,Mode_Choice)
        # time.sleep(0.0015)
        # if mod_cmd:
        #     infostr='Mode:'+str(Mode_Choice)+'Set Ok!!!'
        #     self.logger.loggerinfo(infostr)
        #     return True
        # else:
        #     self.logger.loggererror("Please Check your controller or USB CAN Analysis!!!!")
        #     return False
    
    def Send_Control_Command(self,IDD,Commands):
        
        mod_cmd=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],self.CanAnalysis.yamlDic['Frames_Length'],IDD,self.Datalen,Commands)
        time.sleep(0.0015)
        if mod_cmd:
            infostr='Command:'+str(Commands)+'send Ok!!!'
            self.logger.loggerinfo(infostr)
            return True
        else:
            self.logger.loggererror("Please Check your controller or USB CAN Analysis!!!!")
            return False

    def Send_YAML_COMMAND_To_Motor(self,flag,cmd):
        if flag:
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_left_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_right_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_left_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_right_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
        elif flag==0:
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
        else:
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_left_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_right_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_left_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_right_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
    def Send_Velocity_Driver(self,VelocityData,Left_Right,Driver_Id):
        """
        Unit:RPM/MIN
        """
        command=self.Caculate_Velocity_Command(int(VelocityData),Left_Right)
        self.Send_Control_Command(Driver_Id,command)
        time.sleep(0.0015)
    def Send_Position_Driver(self,PosistionData,Left_Right,Driver_Id):
        command=self.Caculate_Position_Command(int(PosistionData),Left_Right)
        self.Send_Control_Command(Driver_Id,command)
        time.sleep(0.0015)
    def Caculate_Velocity_Command(self,VelocityData,flag):

        BASIC=self.MobileDriver_Command.BASIC_TARGET_COMMAND#(0x23, 0xff, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00)

        NEWCMD=()
        if flag:#steer
            VelocityData=VelocityData*512*4096/1875
        else:
            VelocityData=VelocityData*512*10000/1875
        if VelocityData<0:
            velocity=hex(VelocityData & 0xFFFFFFFF)
            newvelocity=[velocity[i:i+2] for i in range(0,len(velocity), 2)]
            # print newvelocity
            NEWCMD=(int(newvelocity[4],16),int(newvelocity[3],16),int(newvelocity[2],16),int(newvelocity[1],16))+BASIC[4:]
            # print newvelocity,NEWCMD
            return NEWCMD

        elif VelocityData>0:
            velocity=hex(VelocityData).replace('0x','')
            # print velocity
            newvelocity=velocity.zfill(len(velocity)+8-len(velocity))
            # print newvelocity
            tempnewvelocity=[newvelocity[i:i+2] for i in range(0,len(newvelocity), 2)]
            # print tempnewvelocity
            NEWCMD=(int(tempnewvelocity[3],16),int(tempnewvelocity[2],16),int(tempnewvelocity[1],16),int(tempnewvelocity[0],16))+BASIC[5:]
            # print NEWCMD
            return NEWCMD

        else:
            pass

    def Caculate_Position_Command(self,PositionData,flag):
        BASIC=self.MobileDriver_Command.BASIC_TARGET_COMMAND

        NEWCMD=()

        if PositionData<0:

                position=hex(PositionData & 0xFFFFFFFF)
                newposition=[position[i:i+2] for i in range(0,len(position), 2)]
                NEWLEFT=BASIC[:4]+(int(newposition[4],16),int(newposition[3],16),int(newposition[2],16),int(newposition[1],16))
                # print newposition,NEWLEFT
                return NEWCMD

        elif PositionData>0:
            position=hex(PositionData).replace('0x','')
            # print position
            if len(position)>=1:
                newposition=position.zfill(8-len(position))
                tempnewposition=[newposition[i:i+2] for i in range(0,len(newposition), 2)]
                # print tempnewposition
            else:
                pass

            NEWCMD=BASIC[:4]+(int(tempnewposition[3],16),int(tempnewposition[2],16),int(tempnewposition[1],16),int(tempnewposition[0],16))
            # print NEWLEFT
            return NEWCMD

        else:
            pass
def main():
    mpd=MobilePlatformDriver()
    count=1
    flag=0
    mpd.Open_driver_can_Node(0x00000000,1)
    mpd.Enable_Motor_Controller_All()
    # time.sleep(10)
    # print "homing is ok----!!!"
    # while count:
    #     # mpd.Save_Parameter('1')
    #     # time.sleep(1)
    #     if flag==1:
    #         mpd.Send_YAML_COMMAND_To_Motor(3,mpd.MobileDriver_Command.ZERO_VELOCITY)
    #     else:
    #         mpd.Send_YAML_COMMAND_To_Motor(0,mpd.MobileDriver_Command.TEST_VELOCITY_STEERING)
    #         mpd.Send_YAML_COMMAND_To_Motor(1,mpd.MobileDriver_Command.TEST_VELOCITY_WALKING)
    #     count-=1
    # time.sleep(5)
    # mpd.Send_YAML_COMMAND_To_Motor(1,mpd.MobileDriver_Command.ZERO_VELOCITY)
    # mpd.Send_YAML_COMMAND_To_Motor(0,mpd.MobileDriver_Command.ZERO_VELOCITY)
    mpd.CanAnalysis.Can_VCICloseDevice()
if __name__=="__main__":
    main()