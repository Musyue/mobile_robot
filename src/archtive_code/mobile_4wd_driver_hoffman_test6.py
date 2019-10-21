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
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# import matplotlib.pyplot as plt
class AGV4WDICONTROLLER():
    def __init__(self):
        self.mpfh=MobilePlatformDriver()
        rospy.init_node("imu_data_for_mobileplatform")
        self.wheel_R=0.15/2#m
        self.car_length=0.5
        self.car_width=0.395
        self.imu_sub=rospy.Subscriber('/imu_data',Imu,self.Imu_callback)
        self.cmd_vel_sub=rospy.Subscriber('/cmd_vel',Twist,self.CmdVel_callback)
        self.path_sub=rospy.Subscriber('/path_target',Path,self.PathTarget_callback)
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
        self.odemetry_x=0.0
        self.odemetry_y=0.0
        self.odemetry_pha=0.0#3.14
        self.odemetry_beta=0.0
        self.vel_reference=1.0#0.5
        self.reference_x=0
        self.reference_y=0
        self.reference_pha=0
        self.reference_beta=0
        ####hoffamn
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tranformtfs=""
        self.tranformtft=''
        self.trans = self.tfBuffer.lookup_transform(self.tranformtfs, self.tranformtft, rospy.Time())
        self.pose_x=0.0
        self.pose_y=0.
        self.pose_z=0.
        self.pose_quaternion_x=0.
        self.pose_quaternion_y=0.
        self.pose_quaternion_z=0.0
        self.pose_quaternion_w=0.0
        self.roll=0
        self.pitch=0
        self.yaw=0
        self.st=tan(0)
        self.phi=0#gama
        self.index_ref=0
        self.phaRdot=0.08
        self.betaRdot=0
        self.kk=10
        self.limit_steer_rad=10
        self.read_path=loadmat('/data/ros/yue_wk_2019/src/mobile_robot/src/mobile_control/circle_shape_path_2.mat')#figure_eight_path.mat')
        # self.pub_vstar=rospy.Publisher("/vstar",Float64,queue_size=10)
        self.pub_xr=rospy.Publisher("/xr",Float64,queue_size=10)
        self.pub_yr=rospy.Publisher("/yr",Float64,queue_size=10)
        self.pub_target_pha=rospy.Publisher("/target_pha",Float64,queue_size=10)
        self.pub_x=rospy.Publisher("/x",Float64,queue_size=10)
        self.pub_y=rospy.Publisher("/y",Float64,queue_size=10)
        self.pub_real_pha=rospy.Publisher("/real_pha",Float64,queue_size=10)
        
        self.pub_Vfl=rospy.Publisher("/vfl",Float64,queue_size=10)
        self.pub_Vfr=rospy.Publisher("/vfr",Float64,queue_size=10)
        self.pub_Vrl=rospy.Publisher("/vrl",Float64,queue_size=10)
        self.pub_Vrr=rospy.Publisher("/vrr",Float64,queue_size=10)
        self.pub_detafl=rospy.Publisher("/detafl",Float64,queue_size=10)
        self.pub_detafr=rospy.Publisher("/detafr",Float64,queue_size=10)
        # Vfl,Vfr,Vrl,Vrr,detafl,detafr,detarl,detarr
        self.target_path=[]
        self.homing_original_position=[self.mpfh.Driver_steer_encode_fl_original,self.mpfh.Driver_steer_encode_fr_original,self.mpfh.Driver_steer_encode_rl_original,self.mpfh.Driver_steer_encode_rr_original]
    def CmdVel_callback(self,msg):
        # print "msg",msg.linear.x
        self.linear_x=msg.linear.x
        self.linear_y=msg.linear.y
        self.linear_z=msg.linear.z
        self.angular_x=msg.angular.x
        self.angular_y=msg.angular.y
        self.angular_z=msg.angular.z
    def PathTarget_callback(self,msg):
        self.pose_x=msg.pose.position.x
        self.pose_y=msg.pose.position.y
        self.pose_z=msg.pose.position.z
        self.pose_quaternion_x=msg.pose.orientation.x
        self.pose_quaternion_y=msg.pose.orientation.y
        self.pose_quaternion_z=msg.pose.orientation.z
        self.pose_quaternion_w=msg.pose.orientation.w
        orientation_list = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        (self.roll,self.pitch,self.yaw) = euler_from_quaternion (orientation_list)
    def Avage_list(self,listdata,appendata):
        if len(listdata)>10:
            listdata=listdata[1:]
            listdata.append(appendata)
        else:
            listdata.append(appendata)
        return listdata
    def Init_Node(self):
        
        # rospy.init_node("imu_data_for_mobileplatform")
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
        print "----d------",d
        return  d#self.mod_function(d+pi, 2*pi) - pi

    def Caculate_velocity_from_angular_z(self,angular_velocity_z,gamma_rad):
        vel=(angular_velocity_z*self.car_length)/tan(gamma_rad)
        return vel
    def Caculate_velocity_from_RPM(self):

        RPM_fl=self.mpfh.Dec_to_RPM(self.mpfh.Driver_walk_velocity_encode_fl)
        RPM_fr=self.mpfh.Dec_to_RPM(self.mpfh.Driver_walk_velocity_encode_fr)
        RPM_rl=self.mpfh.Dec_to_RPM(self.mpfh.Driver_walk_velocity_encode_rl)
        RPM_rr=self.mpfh.Dec_to_RPM(self.mpfh.Driver_walk_velocity_encode_rr)
        print("RPM_fl",RPM_fl,RPM_fr,RPM_rl,RPM_rr)
        if 0 not in [RPM_fl,RPM_fr,RPM_rl,RPM_rr]:
            Velocity=[(RPM_fl*2*pi*self.wheel_R)/60.0,(RPM_fr*2*pi*self.wheel_R)/60.0,(RPM_rl*2*pi*self.wheel_R)/60.0,(RPM_rr*2*pi*self.wheel_R)/60.0]
            print("------Velocity---------",Velocity)#self.mpfh.Driver_walk_velocity_encode_fl
            return Velocity
        else:
            Velocity=[(RPM_fl*2*pi*self.wheel_R)/60.0,(RPM_fr*2*pi*self.wheel_R)/60.0,(RPM_rl*2*pi*self.wheel_R)/60.0,(RPM_rr*2*pi*self.wheel_R)/60.0]
            print("----some zero in list for velocity---",Velocity)
            return [-1.0*self.vel_reference,self.vel_reference,-1.0*self.vel_reference,self.vel_reference]
            # print "there are velocity error in encode"

    def Caculate_rad_from_position_data(self):
        detafi=self.mpfh.Pos_to_rad(self.mpfh.Driver_steer_encode_fl-self.mpfh.Driver_steer_encode_fl_original)
        detafo=self.mpfh.Pos_to_rad(self.mpfh.Driver_steer_encode_fr-self.mpfh.Driver_steer_encode_fr_original)
        detari=self.mpfh.Pos_to_rad(self.mpfh.Driver_steer_encode_rl-self.mpfh.Driver_steer_encode_rl_original)
        detaro=self.mpfh.Pos_to_rad(self.mpfh.Driver_steer_encode_rr-self.mpfh.Driver_steer_encode_rr_original)
        print("self.mpfh.Driver_steer_encode_fl",self.mpfh.Driver_steer_encode_fl,self.mpfh.Driver_steer_encode_fr,self.mpfh.Driver_steer_encode_rl,self.mpfh.Driver_steer_encode_rr)
        return [detafi,detafo,detari,detaro]
    def caculate_bicycle_model_thetafr_re(self,VA,phadot):
        # print self.linear_x,self.angular_z

        thetafr=atan((phadot*self.car_length)/(2.0*VA))
        thetare=atan(-tan(thetafr))
        return [thetafr,thetare]

    def my_arccot(self,x):
        """
        Using pi/2-atan(x) to caculate arccot
        """
        return pi/2-atan(x)
        # if x>0:
        #     return atan(1/x)+pi-3328842.5883102473,
        # elif x<0:
        #     return atan(1/x)
        # else:
        #     return 0.0
    
    def Set_rad_in_halfpi(self,rad):
        """
        Make rad in 0-pi/2
        """
        if rad<pi/2.0:
            return rad
        else:
            return rad-pi
        # return (rad+pi/2.0)%(pi/2.0) - pi/2.0
    def Set_rad_in_pi(self,rad):
        return (rad+pi)%(pi*2.0) - pi
    def sign(self,x):
        if x!=0.0:
            return x/abs(x)
        else:
            return 0.0
    def hoffman_control(self,point_ref_all,dt):
        self.odemetry_x=self.odemetry_x+self.vel_reference*cos(self.odemetry_pha)*dt
        self.odemetry_y=self.odemetry_y+self.vel_reference*sin(self.odemetry_pha)*dt
        self.odemetry_pha=self.odemetry_pha+self.vel_reference*(1.0/self.car_length)*self.st*dt
        self.odemetry_pha=self.Set_rad_in_pi(self.odemetry_pha)
        e=sqrt((self.odemetry_x-point_ref_all[0][0])**2+(self.odemetry_y-point_ref_all[0][1])**2)
        for i in range(2,len(point_ref_all)):
            etmp=sqrt((self.odemetry_x-point_ref_all[i][0])**2+(self.odemetry_y-point_ref_all[i][1])**2)
            if etmp<e:
                e=etmp
                self.index_ref=i
        print("self.index_ref",self.index_ref)
        point_ref=point_ref_all[self.index_ref]
        ex1=point_ref[0]-self.odemetry_x
        ey1=point_ref[1]-self.odemetry_y
        self.pub_xr.publish(point_ref[0])
        self.pub_yr.publish(point_ref[1])
        self.pub_target_pha.publish(point_ref[2])
        ex2=cos(point_ref[2])
        ey2=sin(point_ref[2])
        sinnn=-1*(ex1*ey2-ey1*ex2)
        e=self.sign(sinnn)*e
        self.phi=point_ref[2]- self.odemetry_pha+atan(self.kk*e/self.vel_reference)
        self.st=tan(self.phi)
        if abs(self.st)>=self.limit_steer_rad:
            self.st=self.limit_steer_rad*self.sign(self.st)
        else:
            self.st=self.st
    def set_array_to_list(self,array_num):
        newtemp=[]
        for i in range(len(array_num)):
            newtemp.append(list(array_num[i]))
        return newtemp
    def hoffman_kinematic_model(self,VC,phi_ref):
        Vfl=VC*sqrt(self.car_length**2+(self.car_length/tan(phi_ref)+self.car_width/2.0)**2)/(self.car_length/tan(phi_ref))
        Vfr=VC*sqrt(self.car_length**2+(self.car_length/tan(phi_ref)-self.car_width/2.0)**2)/(self.car_length/tan(phi_ref))
        Vrl=VC*(self.car_length/tan(phi_ref)+self.car_width/2.0)/(self.car_length/tan(phi_ref))
        Vrr=VC*(self.car_length/tan(phi_ref)-self.car_width/2.0)/(self.car_length/tan(phi_ref))
        detarl=0.0
        detarr=0.0
        detafl=atan(self.car_length/(self.car_length/tan(phi_ref)-self.car_width/2.0))
        detafr=atan(self.car_length/(self.car_length/tan(phi_ref)+self.car_width/2.0))
        self.pub_Vfl.publish(Vfl)
        self.pub_Vfr.publish(Vfr)
        self.pub_Vrl.publish(Vrl)
        self.pub_Vrr.publish(Vrr)
        self.pub_detafl.publish(detafl)
        self.pub_detafr.publish(detafr)
        return [Vfl,Vfr,Vrl,Vrr,detafl,detafr,detarl,detarr]
def main():
   agvobj=AGV4WDICONTROLLER()
   agvobj.Init_Node()
   time.sleep(5)
   ratet=1
   rate=rospy.Rate(ratet)
   zerotime=time.time()
   dt=0

   flg=0
   count=1
   xr=[]
   yr=[]
   x=[]
   y=[]
#    plt.ion() #开启interactive mode 成功的关键函数
#    plt.figure(1)
#    agvobj.add_target()
#    plt.show()
#    postion_list
   speedfl=0
   speedfr=0
   speedrl=0
   speedrr=0
   flagg=1
   flaggg=1
   pathfilename='path'#'pathsmallCirclexythera'
   while not rospy.is_shutdown():
        recevenum=agvobj.mpfh.CanAnalysis.Can_GetReceiveNum(0)
        starttime=time.time()
        print("recevenum----",recevenum)
        if recevenum!=None:
            if flg==0:
                agvobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1.0,1.0,-1.0,1.0],1,agvobj.vel_reference)
                # time.sleep(0.5)
                # agvobj.mpfh.Send_position_to_four_steering_wheel(agvobj.homing_original_position)
                flg=1
            agvobj.mpfh.Read_sensor_data_from_driver()

            pathreference=agvobj.set_array_to_list(agvobj.read_path[pathfilename])
                # print "pathreference",pathreference,len(pathreference)
            agvobj.hoffman_control(pathreference,dt)

            velocity_real_time=agvobj.Caculate_velocity_from_RPM()
            if len(velocity_real_time)!=0:
                rad_real_time=agvobj.Caculate_rad_from_position_data()
                print("velocity_real_time",velocity_real_time)
                print("rad_real_time",rad_real_time)
                Vfi=velocity_real_time[0]
                Vfo=velocity_real_time[1]
                Vri=velocity_real_time[2]
                Vro=velocity_real_time[3]
                detafi=rad_real_time[0]
                detafo=rad_real_time[1]
                detari=rad_real_time[2]
                detaro=rad_real_time[3]
                VC=(agvobj.sign(Vri)*Vri+agvobj.sign(Vro)*Vro)/2.0
                print("VC-------",VC)
                v_deta=agvobj.hoffman_kinematic_model(VC,agvobj.phi)
                print("four velocity and four steer---",v_deta)
                wheel_diretion_flg=[-1.0,1.0,-1.0,1.0]
                wheel_diretion_flg1=[1.0,1.0,1.0,1.0]
                speed_flag=[1.0,-1.0,-1.0,-1.0]
                speedfl=wheel_diretion_flg1[0]*abs(v_deta[0])
                speedfr=wheel_diretion_flg1[1]*abs(v_deta[1])
                speedrl=wheel_diretion_flg1[2]*abs(v_deta[2])
                speedrr=wheel_diretion_flg1[3]*abs(v_deta[3])
                print("speedfl,speedfr,speedrl,speedrr",speedfl,speedfr,speedrl,speedrr)
                agvobj.mpfh.Send_diff_velocity_to_four_walking_wheel(wheel_diretion_flg,speed_flag,speedfl,speedfr,speedrl,speedrr)
                ratation_flag=[-1.0,-1.0,-1.0,-1.0]
                four_steer_degree_theta=v_deta[4:]
                agvobj.mpfh.Send_diff_degree_position_to_four_steering_wheel(ratation_flag,four_steer_degree_theta)

                agvobj.pub_x.publish(agvobj.odemetry_x)
                agvobj.pub_y.publish(agvobj.odemetry_y)
                agvobj.pub_real_pha.publish(agvobj.odemetry_pha)


                # xr.append(pathreference[agvobj.index_ref][0])
                # yr.append(pathreference[agvobj.index_ref][1])
                # x.append(agvobj.odemetry_x)
                # y.append(agvobj.odemetry_y)
                # # print "x,y",x,y
                # # plt.xlim(0., 10.5)
                # # plt.ylim(0., 10.0)
                # plt.plot(xr,yr,'ro',x,y,'bs')
                # plt.draw()
                # plt.pause(0.01)
                count+=1
                print("count",count)
        else:
            agvobj.mpfh.Send_Control_Command(agvobj.mpfh.CanAnalysis.yamlDic['sync_data_ID'], agvobj.mpfh.MobileDriver_Command.ZERO_COMMAND)
            print("---------read data----")
        endtime=time.time()
        dt=endtime-starttime
        # agvobj.

        rate.sleep() 
   agvobj.mpfh.CanAnalysis.Can_VCICloseDevice()
   agvobj.mpfh.Close_driver_can_Node(0x00000000,1)
if __name__=="__main__":
    main()