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
from scipy.io import loadmat
class AGV4WDICONTROLLER():
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
        self.odemetry_pha=0
        self.odemetry_beta=0
        self.vel_reference=0
        self.reference_x=0
        self.reference_y=0
        self.reference_pha=0
        self.reference_beta=0
        self.k1=0
        self.k2=0
        self.k3=0
        self.k4=0
        self.phaRdot=0.08
        self.betaRdot=0
        self.read_path=loadmat('./path.mat')
        
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
    def andiff(self,th1,th2):
        d=th1-th2
        #d = mod(d+pi, 2*pi) - pi;
        print "----d------",d
        return  d#self.mod_function(d+pi, 2*pi) - pi

    def Caculate_velocity_from_angular_z(self,angular_velocity_z,gamma_rad):
        vel=(angular_velocity_z*self.car_length)/tan(gamma_rad)
        return vel
    def Caculate_velocity_from_RPM(self):
        # Velocity=[]

        RPM_fl=self.mpfh.Dec_to_RPM(self.mpfh.Driver_walk_velocity_encode_fl)
        RPM_fr=self.mpfh.Dec_to_RPM(self.mpfh.Driver_walk_velocity_encode_fr)
        RPM_rl=self.mpfh.Dec_to_RPM(self.mpfh.Driver_walk_velocity_encode_rl)
        RPM_rr=self.mpfh.Dec_to_RPM(self.mpfh.Driver_walk_velocity_encode_rr)
        Velocity=[(RPM_fl*2*pi*self.wheel_R)/60.0,(RPM_fr*2*pi*self.wheel_R)/60.0,(RPM_rl*2*pi*self.wheel_R)/60.0,(RPM_rr*2*pi*self.wheel_R)/60.0]
        print "------Velocity---------",Velocity#self.mpfh.Driver_walk_velocity_encode_fl
        return Velocity

    def Caculate_rad_from_position_data(self):
        detafi=self.mpfh.Pos_to_rad(self.mpfh.Driver_steer_encode_fl-self.mpfh.Driver_steer_encode_fl_original)
        detafo=self.mpfh.Pos_to_rad(self.mpfh.Driver_steer_encode_fr-self.mpfh.Driver_steer_encode_fr_original)
        detari=self.mpfh.Pos_to_rad(self.mpfh.Driver_steer_encode_rl-self.mpfh.Driver_steer_encode_rl_original)
        detaro=self.mpfh.Pos_to_rad(self.mpfh.Driver_steer_encode_rr-self.mpfh.Driver_steer_encode_rr_original)
        return [detafi,detafo,detari,detaro]
    def caculate_bicycle_model_thetafr_re(self,VA,phadot):
        # print self.linear_x,self.angular_z

        thetafr=atan((phadot*self.car_length)/(2.0*VA))
        thetare=atan(-tan(thetafr))
        return [thetafr,thetare]

    def my_arccot(self,x):
        return pi/2-atan(x)
        # if x>0:
        #     return atan(1/x)+pi
        # elif x<0:
        #     return atan(1/x)
        # else:
        #     return 0.0
    def caculate_VA_detafr_detare(self,Vfi,Vfo,Vri,Vro,detafi,detafo,detari,detaro):

        detafr=self.my_arccot(0.5*(1/tan(detafr)+1/tan(detafo)))
        detare=self.my_arccot(0.5*(1/tan(detari)+1/tan(detaro)))
        VA_fi=Vfi*sqrt(1+1/4*(tan(detafr)+tan(detare))**2)/(tan(detafr)*(1/sin(detafi)))
        VA_fo=Vfi*sqrt(1+1/4*(tan(detafr)+tan(detare))**2)/(tan(detafr)*(1/sin(detafo)))
        VA_ri=Vfi*sqrt(1+1/4*(tan(detafr)+tan(detare))**2)/(tan(detare)*(1/sin(detari)))
        VA_ro=Vfi*sqrt(1+1/4*(tan(detafr)+tan(detare))**2)/(tan(detare)*(1/sin(detaro)))
        VA=(VA_fi+VA_fo+VA_ri+VA_ro)/4

        return [detafr,detare,VA]
    def caculate_XA_YA_phaA_betaA(sef,dt,VA_detafr_detare):
        # VA_detafr_detare=self.caculate_VA_detafr_detare(Vfi,Vfo,Vri,Vro,detafi,detafo,detari,detaro)
        self.odemetry_beta=self.my_arccot(0.5*(tan(VA_detafr_detare[0])+tan(VA_detafr_detare[1])))
        phaAdot=VA_detafr_detare[2]*cos(self.odemetry_beta)*(tan(VA_detafr_detare[0])-tan(VA_detafr_detare[1]))/self.car_length
        self.odemetry_pha+=phaAdot*dt
        XAdot=VA_detafr_detare[2]*cos(self.odemetry_pha+self.odemetry_beta)
        YAdot=VA_detafr_detare[2]*sin(self.odemetry_pha+self.odemetry_beta)
        self.odemetry_x+=XAdot*dt
        self.odemetry_y+=YAdot*dt
    def caculate_e1_e2_e3_e4(self,XR,YR,phaR,betaR):
        e1=(self.odemetry_x-XR)*cos(self.odemetry_pha)+(self.odemetry_y-YR)*sin(self.odemetry_pha)
        e2=-(self.odemetry_x-XR)*sin(self.odemetry_pha)+(self.odemetry_y-YR)*cos(self.odemetry_pha)
        e3=self.odemetry_pha-phaR
        e4=self.odemetry_beta-betaR
        return [e1,e2,e3,e4]
    def caculate_next_time_VA_phaAdot_betaAdot(self,error):
        # error=self.caculate_e1_e2_e3_e4(XR,YR,phaR,betaR)
        VR=self.vel_reference
        VA=-self.k1*error[0]+VR*cos(error[2])
        phaAdot=self.phaRdot-error[1]*self.k2*VR-self.k3*sin(error[2])
        betaAdot=self.betaRdot-self.k4*error[3]
        return [VA,phaAdot,betaAdot]
    
    def caculate_four_steer_degree_theta(self,temp_fr_re):
        """
        arccot(x)=
        {
            arctan(1/x)+π(x>0)
            arctan(1/x)(x<0)
        }
        """
        # temp_fr_re=self.caculate_bicycle_model_thetafr_re()
        
        temp_theta_fo_fr=(1/tan(temp_fr_re[0]))*(1+(self.car_width/self.car_length)*(tan(temp_fr_re[0])-tan(temp_fr_re[1])))
        temp_theta_fi_fl=(1/tan(temp_fr_re[0]))*(1-(self.car_width/self.car_length)*(tan(temp_fr_re[0])-tan(temp_fr_re[1])))
        temp_theta_ro_rr=(1/tan(temp_fr_re[1]))*(1+(self.car_width/self.car_length)*(tan(temp_fr_re[0])-tan(temp_fr_re[1])))
        temp_theta_ri_rl=(1/tan(temp_fr_re[1]))*(1-(self.car_width/self.car_length)*(tan(temp_fr_re[0])-tan(temp_fr_re[1])))
        theta_fo_fr=self.my_arccot(temp_theta_fo_fr)
        theta_fi_fl=self.my_arccot(temp_theta_fi_fl)
        theta_ro_rr=self.my_arccot(temp_theta_ro_rr)
        theta_ri_rl=self.my_arccot(temp_theta_ri_rl)
        return [temp_theta_fi_fl,theta_fo_fr,temp_theta_ri_rl,temp_theta_ro_rr]

    def caculate_four_walk_motor_velocity(self,NewVA,temp_theta_fl_fr_rl_rr,temp_fr_re):
        # temp_theta_fl_fr_rl_rr=self.caculate_four_steer_degree_theta()
        # temp_fr_re=self.caculate_bicycle_model_thetafr_re()

        v1_fr_fo=(NewVA*tan(temp_fr_re[0])*(1/sin(temp_theta_fl_fr_rl_rr[1])))/sqrt(1+(1/4)*(tan(temp_fr_re[0])+tan(temp_fr_re[1]))**2)
        v2_fl_fi=(NewVA*tan(temp_fr_re[0])*(1/sin(temp_theta_fl_fr_rl_rr[0])))/sqrt(1+(1/4)*(tan(temp_fr_re[0])+tan(temp_fr_re[1]))**2)
        v3_rr_ro=(NewVA*tan(temp_fr_re[1])*(1/sin(temp_theta_fl_fr_rl_rr[3])))/sqrt(1+(1/4)*(tan(temp_fr_re[0])+tan(temp_fr_re[1]))**2)
        v4_rl_ri=(NewVA*tan(temp_fr_re[1])*(1/sin(temp_theta_fl_fr_rl_rr[2])))/sqrt(1+(1/4)*(tan(temp_fr_re[0])+tan(temp_fr_re[1]))**2)
        return [v2_fl_fi,v1_fr_fo,v4_rl_ri,v3_rr_ro]

def main():
   agvobj=AGV4WDICONTROLLER()
   agvobj.Init_Node()
   ratet=10
   rate=rospy.Rate(ratet)
   zerotime=time.time()
   dt=0
   agvobj.set_pdemetry_x(x0[0])
   agvobj.set_pdemetry_y(x0[1])
   agvobj.set_pdemetry_theta(x0[2])
   VR=1.0
   count=0
   while not rospy.is_shutdown():
        recevenum=agvobj.mpfh.CanAnalysis.Can_GetReceiveNum(0)
        starttime=time.time()
        # print "recevenum",recevenum
        if recevenum!=None:
            if flg==0:
                agvobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1,1,-1,1],1,VR)
                flg=1
            agvobj.mpfh.Read_sensor_data_from_driver()
            pathreference=list(agvobj.read_path['path'][count])
            #####[1] caculate xA,yA,phaA,betaA#########
            #### first Vfi,Vfo,Vri,Vro,detafi,detafo,detari,detaro
            velocity_real_time=agvobj.Caculate_velocity_from_RPM()
            rad_real_time=agvobj.Caculate_rad_from_position_data()
            ####sencond caculate VA detafr detare
            Vfi=velocity_real_time[0]
            Vfo=velocity_real_time[1]
            Vri=velocity_real_time[2]
            Vro=velocity_real_time[3]
            detafi=rad_real_time[0]
            detafo=rad_real_time[1]
            detari=rad_real_time[2]
            detaro=rad_real_time[3]
            VA_detafr_detare=agvobj.caculate_VA_detafr_detare(Vfi,Vfo,Vri,Vro,detafi,detafo,detari,detaro)
            #####third XA_YA_phaA_betaA
            XA_YA_phaA_betaA=agvobj.caculate_XA_YA_phaA_betaA(dt,VA_detafr_detare)
            #####fourth e1_e2_e3_e4
            XR=pathreference[0]
            YR=pathreference[1]
            phaR=pathreference[2]
            betaR=0
            error=agvobj.caculate_e1_e2_e3_e4(XR,YR,phaR,betaR)
            ######fifth caculate_next_time_VA_phaAdot_betaAdot
            next_time_VA_phaAdot_betaAdot=agvobj.caculate_next_time_VA_phaAdot_betaAdot(error)
            ####sixth new thetafr_re
            new_VA=next_time_VA_phaAdot_betaAdot[0]
            new_phadot=next_time_VA_phaAdot_betaAdot[1]
            new_model_thetafr_re=agvobj.caculate_bicycle_model_thetafr_re(new_VA,new_phadot)
            #### seventh caculate_four_steer_degree_theta
            #### temp_theta_fi_fl,theta_fo_fr,temp_theta_ri_rl,temp_theta_ro_rr
            four_steer_degree_theta=agvobj.caculate_four_steer_degree_theta(new_model_thetafr_re)
            #### eighth four_walk_motor_velocity
            #### v2_fl_fi,v1_fr_fo,v4_rl_ri,v3_rr_ro
            four_walk_motor_velocity=agvobj.caculate_four_walk_motor_velocity(new_VA,four_steer_degree_theta,new_model_thetafr_re)
            #### finally control mobile platform
            wheel_diretion_flg=[-1,1,-1,1]
            speed_flag=[-1,-1,-1,-1]
            speedfl=four_walk_motor_velocity[0]
            speedfr=four_walk_motor_velocity[1]
            speedrl=four_walk_motor_velocity[2]
            speedrr=four_walk_motor_velocity[3]
            agvobj.mpfh.Send_diff_velocity_to_four_walking_wheel(wheel_diretion_flg,speed_flag,speedfl,speedfr,speedrl,speedrr)
            ratation_flag=[-1.0,-1.0,-1.0,-1.0]
            agvobj.mpfh.Send_diff_degree_position_to_four_steering_wheel(ratation_flag,four_steer_degree_theta)
        else:
            agvobj.mpfh.Send_Control_Command( agvobj.mpfh.CanAnalysis.yamlDic['sync_data_ID'], agvobj.mpfh.MobileDriver_Command.ZERO_COMMAND)
        endtime=time.time()
        dt=endtime-starttime
        count+=1
        rate.sleep() 
if __name__=="__main__":
    main()