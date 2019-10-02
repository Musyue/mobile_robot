#! /usr/bin/env python
# coding=utf-8
import rospy
import sys
from std_msgs.msg import String,Float64,Bool,Int64MultiArray
from sensor_msgs.msg import Imu
import time 
from math import *
import numpy as np
from mobile_control.mobileplatform_driver_steptech import *
from geometry_msgs.msg import Twist
class IMUDATAINTERGAL():
    def __init__(self):
        self.mpfh=MobilePlatformDriver()
        self.wheel_R=0.15/2#m
        self.car_length=0.5
        self.car_width=0.395
        self.imu_sub=rospy.Subscriber('/imu_data',Imu,self.Imu_callback)
        self.cmd_vel_sub=rospy.Subscriber('/cmd_vel',Twist,self.CmdVel_callback)
        self.ImuOrientation=()
        self.ImuAngularvelocity=()
        self.ImuLinearAcceleration=()
        self.ImuOrientationCovariance=[]
        self.ImuAngularvelocityCovariance=[]
        self.ImuLinearAccelerationCovariance=[]
        self.linear_x=0.00001
        self.linear_y=0
        self.linear_z=0
        self.angular_x=0
        self.angular_y=0
        self.angular_z=0.00001
        self.speed_rotation=[]
        self.odemetry_x=0
        self.odemetry_y=0
        self.odemetry_theta=0
        self.odemetry_vel=0.000001
        self.homing_original_position=[self.mpfh.Driver_steer_encode_fl_original,self.mpfh.Driver_steer_encode_fr_original,self.mpfh.Driver_steer_encode_rl_original,self.mpfh.Driver_steer_encode_rr_original]
    def CmdVel_callback(self,msg):
        # print "msg",msg.linear.x
        self.linear_x=msg.linear.x
        self.linear_y=msg.linear.y
        self.linear_z=msg.linear.z
        self.angular_x=msg.angular.x
        self.angular_y=msg.angular.y
        self.angular_z=msg.angular.z

    def Avage_list(self,listdata,appendata):
        if len(listdata)>10:
            listdata=listdata[1:]
            listdata.append(appendata)
        else:
            listdata.append(appendata)
        return listdata
    def Init_Node(self):
        
        rospy.init_node("imu_data_for_mobileplatform")
        self.mpfh.Init_can()
        self.mpfh.Open_driver_can_Node(0x00000000,1)
        self.mpfh.Enable_Motor_Controller_All()
        self.mpfh.Send_trapezoid_Velocity(2500)
    def Imu_callback(self,msg):
        self.ImuOrientation=(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
        self.ImuAngularvelocity=(msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z)
        self.ImuLinearAcceleration=(msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z)
        self.ImuOrientationCovariance=msg.orientation_covariance
        self.ImuAngularvelocityCovariance=msg.angular_velocity_covariance
        self.ImuLinearAccelerationCovariance=msg.linear_acceleration_covariance
    def set_pdemetry_vel(self,vel):
        self.odemetry_vel=vel
    def set_pdemetry_x(self,x):
        self.odemetry_x=x
    def set_pdemetry_y(self,y):
        self.odemetry_y=y
    def set_pdemetry_theta(self,theta):
        self.odemetry_theta=theta  
    def Bycycle_Model(self,Vel,Gamma_rad,dt):
        # Vel=0.000001
        # theta=0
        # x=0
        # y=0
        # dt=0.00000001
        
        if Vel!=0:
            # starttime=time.time()
            # print imuobj.ImuOrientation
            # print "starttime",starttime
            thetastar=(tan(Gamma_rad)*Vel)/self.car_length
            print "bicycle thetastar",thetastar
            self.odemetry_theta+=thetastar*dt
            xstar=Vel*cos(self.odemetry_theta)
            ystar=Vel*sin(self.odemetry_theta)
            print "bicycle xstar,ystar",xstar,ystar
            self.odemetry_x+=xstar*dt
            self.odemetry_y+=ystar*dt
            print "bicycle x,y,theata",self.odemetry_x,self.odemetry_y,self.odemetry_theta
            # endtime=time.time() 
            # dt=endtime-starttime
            # print "dt",dt
        # print imuobj.mpfh.Driver_walk_velocity_encode_fl
        # return [x,y,theta]
        # else:
        #     pass
    def andiff(self,th1,th2):
        d=th1-th2
        #d = mod(d+pi, 2*pi) - pi;
        print "----d------",d
        return  d#self.mod_function(d+pi, 2*pi) - pi
    def mod_function(self,a,m):
        return a - m*int(a/m)

    def Caculate_velocity_from_angular_z(self,angular_velocity_z,gamma_rad):
        vel=(angular_velocity_z*self.car_length)/tan(gamma_rad)
        return vel
    def Caculate_velocity_from_RPM(self):
        # Velocity=[]
        if self.mpfh.Driver_walk_velocity_encode_fl!=0 and self.mpfh.Driver_walk_velocity_encode_fr!=0 and self.mpfh.Driver_walk_velocity_encode_rl!=0 and self.mpfh.Driver_walk_velocity_encode_rr!=0:
            # print self.mpfh.Driver_walk_velocity_encode_fl
            # print self.mpfh.Driver_walk_velocity_encode_fr
            # print self.mpfh.Driver_walk_velocity_encode_rl
            # print self.mpfh.Driver_walk_velocity_encode_rr
            RPM_fl=self.mpfh.Dec_to_RPM(self.mpfh.Driver_walk_velocity_encode_fl)
            RPM_fr=self.mpfh.Dec_to_RPM(self.mpfh.Driver_walk_velocity_encode_fr)
            RPM_rl=self.mpfh.Dec_to_RPM(self.mpfh.Driver_walk_velocity_encode_rl)
            RPM_rr=self.mpfh.Dec_to_RPM(self.mpfh.Driver_walk_velocity_encode_rr)
            Velocity=[(RPM_fl*2*pi*self.wheel_R)/60.0,(RPM_fr*2*pi*self.wheel_R)/60.0,(RPM_rl*2*pi*self.wheel_R)/60.0,(RPM_rr*2*pi*self.wheel_R)/60.0]
            print "------Velocity---------",Velocity#self.mpfh.Driver_walk_velocity_encode_fl
            if max(Velocity)-min(Velocity)>10.0:
                return 0.0
            else:
                print "From Wheel Encode Velocity-------",(abs(Velocity[0])+abs(Velocity[1])+abs(Velocity[2])+abs(Velocity[3]))/4
                return (abs(Velocity[0])+abs(Velocity[1])+abs(Velocity[2])+abs(Velocity[3]))/4
        else:
            return 0.0
    def Control_mobile_to_one_target(self,x,y,theta,Kv,Kh,dt):
        """
        World coordinate:[x,y,theta]
        """

        Vstar=Kv*sqrt((x-self.odemetry_x)**2+(y-self.odemetry_y)**2)
        print "self.odemetry_x",self.odemetry_x
        print "x-self.odemetry_x,y-self.odemetry_y",x-self.odemetry_x,y-self.odemetry_y
        thetastar=atan((y-self.odemetry_y)/(x-self.odemetry_x))
        gammastar=Kh*self.andiff(thetastar,theta)#Kh>0
        print "thetastar,theta",thetastar,theta
        self.Bycycle_Model(Vstar,gammastar,dt)
        print "gammastar",gammastar,Vstar,thetastar
        # self.mpfh.Send_same_velocity_to_four_walking_wheel([-1,1,-1,1],1,Vstar)
        # self.mpfh.Send_same_degree_position_to_four_steering_wheel([1.0,-1.0,-0.0,0.0],gammastar)
    def Cmd_vel_control_for_test(self):

        if self.linear_x!=0 or self.angular_z!=0:
            if self.linear_x!=0:
                #print self.mpfh.Driver_steer_encode_fl,abs(self.mpfh.Driver_steer_encode_fl-self.mpfh.Driver_steer_encode_fl_original)
                if abs(self.mpfh.Driver_steer_encode_fl-self.mpfh.Driver_steer_encode_fl_original)>100 or abs(self.mpfh.Driver_steer_encode_fr-self.mpfh.Driver_steer_encode_fr_original)>100 or abs(self.mpfh.Driver_steer_encode_rl-self.mpfh.Driver_steer_encode_rl_original)>100 or abs(self.mpfh.Driver_steer_encode_rr-self.mpfh.Driver_steer_encode_rr_original)>100: 
                    self.mpfh.Send_position_to_four_steering_wheel(self.homing_original_position)
                if self.linear_x==self.linear_x:
                    self.mpfh.Send_same_velocity_to_four_walking_wheel([-1,1,-1,1],1,self.linear_x)
                    time.sleep(0.03)
            if self.angular_z!=0:
    
                self.mpfh.Send_same_degree_position_to_four_steering_wheel([1.0,-1.0,-1.0,1.0],pi/4)
                speed=self.Caculate_velocity_from_angular_z(self.angular_z,pi/4)
                print "speed",speed
                if self.angular_z==self.angular_z and abs(speed)>0.0001:
                    self.mpfh.Send_same_velocity_to_four_walking_wheel([-1.0,-1.0,-1.0,-1.0],1,speed)
        else:
            # print "Velocity is Zero"
            time.sleep(0.00015)
            self.mpfh.Send_same_velocity_to_four_walking_wheel([-1,1,-1,1],1,0)

def main():
   imuobj=IMUDATAINTERGAL()
   imuobj.Init_Node()
   ratet=1
   rate=rospy.Rate(ratet)
   zerotime=time.time()
#    theta=0#(x,y,theta)
   gama=0#(roation angular)
   v=0.1331
   dt=0
   x=0
   y=0
   flg=0
   xg=[5,5]
   x0=[8,5,0]
   imuobj.set_pdemetry_x(x0[0])
   imuobj.set_pdemetry_y(x0[1])
   imuobj.set_pdemetry_theta(x0[2])
   while not rospy.is_shutdown():
        recevenum=imuobj.mpfh.CanAnalysis.Can_GetReceiveNum(0)
        starttime=time.time()
        # print "recevenum",recevenum
        if recevenum!=None:
            # if flg==0:
            #     imuobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1,1,-1,1],1,1.0)
            #     flg=1
            # print imuobj.linear_x
            imuobj.mpfh.Read_sensor_data_from_driver()
            # imuobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1.0,-1.0,-1.0,-1.0],1,0.5)
            # kk=imuobj.Cmd_vel_control_for_test()

            # Vel=imuobj.Caculate_velocity_from_RPM()
            # imuobj.Bycycle_Model(Vel,0.0)
            if dt!=0:
                print "----dt---",dt
                imuobj.Control_mobile_to_one_target(xg[0],xg[1],0.0,0.5,0.5,dt)
        else:
            imuobj.mpfh.Send_Control_Command( imuobj.mpfh.CanAnalysis.yamlDic['sync_data_ID'], imuobj.mpfh.MobileDriver_Command.ZERO_COMMAND)
        endtime=time.time()
        dt=endtime-starttime
        rate.sleep() 
if __name__=="__main__":
    main()