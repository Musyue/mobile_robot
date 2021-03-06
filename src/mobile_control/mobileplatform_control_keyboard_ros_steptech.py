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
from mobile_control.mobileplatform_driver_steptech import *
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
        self.fl_abs_encode=self.MobileControl.CanAnalysis.yamlDic['Homing_driver_fix_encoder_data']['fl']
        self.fr_abs_encode=self.MobileControl.CanAnalysis.yamlDic['Homing_driver_fix_encoder_data']['fr']
        self.rl_abs_encode=self.MobileControl.CanAnalysis.yamlDic['Homing_driver_fix_encoder_data']['rl']
        self.rr_abs_encode=self.MobileControl.CanAnalysis.yamlDic['Homing_driver_fix_encoder_data']['rr']

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
        self.Driver_steer_velocity_encode_fl=0
        self.Driver_steer_velocity_encode_fr=0
        self.Driver_steer_velocity_encode_rl=0
        self.Driver_steer_velocity_encode_rr=0

        self.Driver_steer_encode_fl_original=self.fl_abs_encode
        self.Driver_steer_encode_fr_original=self.fr_abs_encode
        self.Driver_steer_encode_rl_original=self.rl_abs_encode
        self.Driver_steer_encode_rr_original=self.rr_abs_encode
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
    def Judge_data_NOT_POSITIVE_OR_Negative(self,octdata):
        bindata=list(bin(octdata))
        if bindata[2]=='1':
            return True
        else:
            return False
    def OCT_List_To_Oct_Four(self,Octlist,flag):
        """
        flag=1:position
        flag=0:velocity
        """
        # print "Hexstrlist",Hexstrlist
        temp=[]
        for i in Octlist:
            temp.append(i)
        if flag:
            if self.Judge_data_NOT_POSITIVE_OR_Negative(temp[7]):
                return -1*(~(temp[7]<<24|temp[6]<<16|temp[5]<<8|temp[4]-1)&0xFFFFFFFF)
            else:
                return ((temp[7]<<24|temp[6]<<16|temp[5]<<8|temp[4])&0xFFFFFFFF)
        elif flag==0:
            if self.Judge_data_NOT_POSITIVE_OR_Negative(temp[3]):
                return -1*(~(temp[3]<<24|temp[2]<<16|temp[1]<<8|temp[0]-1)&0xFFFFFFFF)
            else:
                return ((temp[3]<<24|temp[2]<<16|temp[1]<<8|temp[0])&0xFFFFFFFF)
    def New_Read_Encoder_data_From_ABS_Encoder(self,RecNum):

        position_data_walk_vel_fl=self.MobileControl.Send_Control_Command(self.MobileControl.CanAnalysis.yamlDic['sync_data_ID'],self.MobileControl.MobileDriver_Command.ZERO_COMMAND)
        # time.sleep(0.0015)

        ret,kk=self.MobileControl.CanAnalysis.Can_New_Receive(0,RecNum)
        time.sleep(0.05)
        if ret:
            for i in range(RecNum):
                # if kk[i].ID==0x181:
                #     print "walking fl valid model and status  ID",hex(kk[i].ID)
                #     print "walking fl valid model and status  ID--------",self.List_to_HEXList(list(kk[i].Data))

                # if kk[i].ID==0x182:
                #     print "walking fr valid model and status ID",hex(kk[i].ID)
                #     print "walking fr valid model and status--------",self.List_to_HEXList(list(kk[i].Data))
                #     print "walking fr hexdata velocity --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),0)

                # if kk[i].ID==0x183:
                #     print "walking rl valid model and status ID",hex(kk[i].ID)
                #     print "walking rl valid model and status--------",self.List_to_HEXList(list(kk[i].Data))
                #     print "walking rl hexdata velocity --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),0)

                # if kk[i].ID==0x184:
                #     print "walking rr valid model and status ID",hex(kk[i].ID)
                #     print "walking rr valid model and status--------",self.List_to_HEXList(list(kk[i].Data))
                #     print "walking rr hexdata velocity --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),0)

                # if kk[i].ID==0x185:
                #     print "steering fl valid model and status ID",hex(kk[i].ID)
                #     print "steering fl valid model and status--------",self.List_to_HEXList(list(kk[i].Data))
                #     print "steering fl hexdata position --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)

                # if kk[i].ID==0x186:
                #     print "steering fr valid model and status ID",hex(kk[i].ID)
                #     print "steering fr valid model and status--------",self.List_to_HEXList(list(kk[i].Data))
                #     print "steering fr hexdata position --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)

                # if kk[i].ID==0x187:
                #     print "steering rl valid model and status ID",hex(kk[i].ID)
                #     print "steering rl valid model and status--------",self.List_to_HEXList(list(kk[i].Data))
                #     print "steering rl hexdata position --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)

                # if kk[i].ID==0x188:
                #     print "steering rr valid model and status ID",hex(kk[i].ID)
                #     print "steering rr valid model and status--------",self.List_to_HEXList(list(kk[i].Data))
                #     print "steering rr hexdata position --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)

                if kk[i].ID==0x281:
                    # print "walking fl data ID",hex(kk[i].ID)
                    # print "walking fl hexdata--------",self.List_to_HEXList(list(kk[i].Data))
                    # print "walking fl hexdata velocity --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),0)
                    self.Driver_walk_velocity_encode_fl=self.OCT_List_To_Oct_Four(list(kk[i].Data),0)
                if kk[i].ID==0x282:
                    # print "walking fr data ID",hex(kk[i].ID)
                    # print "walking fr hexdata--------",self.List_to_HEXList(list(kk[i].Data))
                    # print "walking fr hexdata velocity --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),0)
                    self.Driver_walk_velocity_encode_fr=self.OCT_List_To_Oct_Four(list(kk[i].Data),0)
                if kk[i].ID==0x283:
                    # print "walking rl data ID",hex(kk[i].ID)
                    # print "walking rl hexdata--------",self.List_to_HEXList(list(kk[i].Data))
                    # print "walking rl hexdata velocity --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),0)
                    self.Driver_walk_velocity_encode_rl=self.OCT_List_To_Oct_Four(list(kk[i].Data),0)
                if kk[i].ID==0x284:
                    # print "walking rr data ID",hex(kk[i].ID)
                    # print "walking rr hexdata--------",self.List_to_HEXList(list(kk[i].Data))
                    # print "walking rr hexdata velocity --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),0)
                    self.Driver_walk_velocity_encode_rr=self.OCT_List_To_Oct_Four(list(kk[i].Data),0)
                if kk[i].ID==0x285:
                    # print "steering fl data ID",hex(kk[i].ID)
                    # print "steering fl hexdata--------",self.List_to_HEXList(list(kk[i].Data))
                    # print "steering fl hexdata position --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                    self.Driver_steer_encode_fl=self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                if kk[i].ID==0x286:
                    # print "steering fr data ID",hex(kk[i].ID)
                    # print "steering fr hexdata--------",self.List_to_HEXList(list(kk[i].Data))
                    # print "steering fr hexdata position --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                    self.Driver_steer_encode_fr=self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                if kk[i].ID==0x287:
                    # print "steering rl data ID",hex(kk[i].ID)
                    # print "steering rl hexdata--------",self.List_to_HEXList(list(kk[i].Data))
                    # print "steering rl hexdata position --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                    self.Driver_steer_encode_rl=self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                if kk[i].ID==0x288:
                    # print "steering rr data ID",hex(kk[i].ID)
                    # print "steering rr hexdata--------",self.List_to_HEXList(list(kk[i].Data))
                    # print "steering rr hexdata position --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                    self.Driver_steer_encode_rr=self.OCT_List_To_Oct_Four(list(kk[i].Data),1)

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
    def rad_to_pulse(self,rad):
        #rad 弧度
        return (rad*(220*1024*4.0)/(2.0*pi))

    def output_pulse_position_control(self,flag_list,degree_tar):
        """
        flag_list=[]
        """
        oldpusle_fl=self.Driver_steer_encode_fl_original
        oldpusle_fr=self.Driver_steer_encode_fr_original
        oldpusle_rl=self.Driver_steer_encode_rl_original
        oldpusle_rr=self.Driver_steer_encode_rr_original

        return [oldpusle_fl+flag_list[0]*self.rad_to_pulse(degree_tar),oldpusle_fr+flag_list[1]*self.rad_to_pulse(degree_tar),oldpusle_rl+flag_list[2]*self.rad_to_pulse(degree_tar),oldpusle_rr+flag_list[3]*self.rad_to_pulse(degree_tar)]

    def Init_mobile_driver(self):
        self.MobileControl.Open_driver_can_Node(0x00000000,1)
        self.MobileControl.Enable_Motor_Controller_All()
        self.MobileControl.Send_trapezoid_Velocity(2500)
        # self.MobileControl.Enable_Walking_Controller()
    
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
    
    mpfh.MobileControl.Init_can()
    start=time.time()
    mpfh.Init_mobile_driver()
    end=time.time()
    print "end---",end-start
    # time.sleep(3)
# mpfh.Init_mobile_driver()
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

    ratet=10

    rate = rospy.Rate(ratet)

    
    speedBindings={
        'q':(1.1,1.05),
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
    turn = 0.01
    flg=1
    flag=1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    print mpfh.strmessage
    print mpfh.vels(speed,turn)

    while not rospy.is_shutdown():
        if 0 not in [mpfh.fl_abs_encode,mpfh.fr_abs_encode,mpfh.rl_abs_encode,mpfh.rr_abs_encode]:
            if flg:
                OutputPulse=[mpfh.fl_abs_encode,mpfh.fr_abs_encode,mpfh.rl_abs_encode,mpfh.rr_abs_encode]
                print "PositionData:",OutputPulse
                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[0]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo3'])
                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[1]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo3'])
                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[2]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo3'])
                mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[3]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo3'])
                flg=0
        recevenum=mpfh.MobileControl.CanAnalysis.Can_GetReceiveNum(0)

        if recevenum!=None:
            mpfh.New_Read_Encoder_data_From_ABS_Encoder(recevenum)

            key = readchar.readkey()
            # if flag:
            #     mpfh.set_original_sensor_data([mpfh.Driver_steer_encode_fl,mpfh.Driver_steer_encode_fr,mpfh.Driver_steer_encode_rl,mpfh.Driver_steer_encode_rr])#([mpfh.Driver_steer_encode_fl,mpfh.Driver_steer_encode_fr,mpfh.Driver_steer_encode_rl,mpfh.Driver_steer_encode_rr])
            #     flag=0
            if key==None:
                continue
            print "key----",key
            # key=0
            # 运动控制方向键（1：正方向，-1负方向）
            if key in moveBindings.keys():
                if key=='a':
                    VelocityData= mpfh.caculate_velocity(-1.0*speed)#all motor 45degree velocity
                    print "VelocityData:RPM/Min",VelocityData
                    mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_right_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_right_rpdo']['rpdo']['rpdo3'])
                elif key =='d':
                    VelocityData= mpfh.caculate_velocity(speed)
                    print "VelocityData:RPM/Min",VelocityData
                    mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_right_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_right_rpdo']['rpdo']['rpdo3'])
                elif key=='i':
                    VelocityData= mpfh.caculate_velocity(-1.0*speed)
                    print "VelocityData:RPM/Min",VelocityData
                    mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Velocity_Driver(1*int(VelocityData),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_right_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Velocity_Driver(1*int(VelocityData),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_right_rpdo']['rpdo']['rpdo3'])
                
                elif key ==',':
                    VelocityData= mpfh.caculate_velocity(speed)
                    print "VelocityData:RPM/Min",VelocityData
                    mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Velocity_Driver(1*int(VelocityData),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_right_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Velocity_Driver(-1*int(VelocityData),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Velocity_Driver(1*int(VelocityData),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_right_rpdo']['rpdo']['rpdo3'])
                elif key =='j':
                    OutputPulse=mpfh.output_pulse_position_control([-1.0,-1.0,-1.0,-1.0],pi/2)
                    print "PositionData:",OutputPulse
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[0]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[1]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[2]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[3]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo3'])
                elif key =='l':
                    OutputPulse=mpfh.output_pulse_position_control([1.0,1.0,1.0,1.0],pi/2)
                    print "PositionData:",OutputPulse
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[0]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[1]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[2]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[3]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo3'])
                elif key == 'u':
                    OutputPulse=mpfh.output_pulse_position_control([-1.0,-1.0,-1.0,-1.0],pi/4)
                    print "PositionData:",OutputPulse
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[0]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[1]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[2]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[3]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo3'])
                elif key == 'o':
                    OutputPulse=mpfh.output_pulse_position_control([1.0,1.0,1.0,1.0],pi/4)
                    print "PositionData:",OutputPulse
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[0]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[1]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[2]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[3]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo3'])
                elif key == 'm':
                    OutputPulse=mpfh.output_pulse_position_control([-1.0,1.0,1.0,-1.0],turn)
                    print "PositionData",OutputPulse
                    fl_data=input("fl_data:")
                    rl_data=input("rl_data:")
                    fr_data=input("fr_data:")
                    rr_data=input("rr_data:")
                    mpfh.MobileControl.Send_Position_Driver(int(fl_data),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(fr_data),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(rl_data),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(rr_data),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo3'])
                    # mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[0]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo3'])
                    # mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[1]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo3'])
                    # mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[2]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo3'])
                    # mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[3]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo3'])
                
                elif key == 'r':
                    OutputPulse=mpfh.output_pulse_position_control([1.0,-1.0,-1.0,1.0],pi/4)
                    print "PositionData",OutputPulse
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[0]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[1]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[2]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[3]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo3'])
                elif key == '.':
                    OutputPulse=mpfh.output_pulse_position_control([-1.0,1.0,1.0,-1.0],turn)
                    print "PositionData:",OutputPulse
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[0]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[1]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[2]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[3]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo3'])
                elif key == 'b':
                    OutputPulse=mpfh.output_pulse_position_control([.0,.0,.0,.0],turn)
                    print "PositionData:",OutputPulse
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[0]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[1]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[2]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo3'])
                    mpfh.MobileControl.Send_Position_Driver(int(OutputPulse[3]),mpfh.MobileControl.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo3'])
                else:
                    pass
                count = 0
            # 速度修改键
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]  # 线速度增加0.1倍
                turn = turn * speedBindings[key][1]    # 角速度增加0.1倍
                count = 0
                if speed>=20:
                    speed=0.1
                print mpfh.vels(speed,turn)
                if (status == 14):
                    print mpfh.strmessage
                status = (status + 1) % 15
            # 停止键
            elif key == ' ' or key =='k':
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
                # circle_or_line_flag=input("Num 1 or 2:")
                # if circle_or_line_flag==2:
                #     mpfh.MobileControl.Send_Velocity_Driver(-1*int(control_speed),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_left_rpdo']['rpdo']['rpdo3'])
                #     mpfh.MobileControl.Send_Velocity_Driver(-1*int(control_speed),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_right_rpdo']['rpdo']['rpdo3'])
                #     mpfh.MobileControl.Send_Velocity_Driver(-1*int(control_speed),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_left_rpdo']['rpdo']['rpdo3'])
                #     mpfh.MobileControl.Send_Velocity_Driver(-1*int(control_speed),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_right_rpdo']['rpdo']['rpdo3'])
                #     # speed = 0
                # else:
                mpfh.MobileControl.Send_Velocity_Driver(-1*int(control_speed),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_left_rpdo']['rpdo']['rpdo3'])
                mpfh.MobileControl.Send_Velocity_Driver(1*int(control_speed),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_right_rpdo']['rpdo']['rpdo3'])
                mpfh.MobileControl.Send_Velocity_Driver(-1*int(control_speed),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_left_rpdo']['rpdo']['rpdo3'])
                mpfh.MobileControl.Send_Velocity_Driver(1*int(control_speed),0,mpfh.MobileControl.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_right_rpdo']['rpdo']['rpdo3'])
                
                # turn = 0
            elif key == 'y' :
                speed = 0.2
                turn = 0.1
            else:
                count = count + 1
                if (key == '\x03'):
                    break
        else:
            recevenum=mpfh.MobileControl.CanAnalysis.Can_GetReceiveNum(0)
        rate.sleep()
        # else:
        #     print "wait data from driver data from feedback topic ----------"
        #     time.sleep(0.5)
    # except:
    #     print e

    # finally:

    #     # time.sleep(3)
    #     print "key board control over-------"
    #     # twist = Twist()
    #     # twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    #     # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    #     # pub.publish(twist)


    mpfh.MobileControl.CanAnalysis.Can_VCICloseDevice()
    mpfh.MobileControl.Close_driver_can_Node(0x00000000,1)
if __name__=="__main__":
    main()