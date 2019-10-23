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
from geometry_msgs.msg import Twist,Pose
from scipy.io import loadmat
import tf2_ros
from nav_msgs.msg import Path
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
        self.path_sub=rospy.Subscriber('/mobile_base_path',Path,self.PathTarget_callback)
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
        self.odemetry_x=0.0#self.trans.transform.translation.x
        self.odemetry_y=0.0#self.trans.transform.translation.y
        self.odemetry_pha=0.0#3.14
        self.odemetry_beta=0.0
        self.vel_reference=0.25#1.5#0.5
        self.reference_x=0
        self.reference_y=0
        self.reference_pha=0
        self.reference_beta=0
        ####hoffamn
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tranformtfs="map"
        self.tranformtft='base_link'
        # self.trans = self.tfBuffer.lookup_transform(self.tranformtfs, self.tranformtft, rospy.Time())
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
        self.kk=0.1
        self.limit_steer_rad=10
        self.path_all=[]
        # self.read_path=loadmat('/data/ros/yue_wk_2019/src/mobile_robot/src/mobile_control/circle_shape_path_2.mat')#figure_eight_path.mat')
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
        self.pub_angular_error=rospy.Publisher("/angular_phi",Float64,queue_size=10)
        self.pub_angular_e=rospy.Publisher("/angular_e",Float64,queue_size=10)
        self.pub_error=rospy.Publisher("/distance_error",Float64,queue_size=10)
        self.pub_pha_error=rospy.Publisher("/pha_error",Float64,queue_size=10)
        self.pub_detafl=rospy.Publisher("/detafl",Float64,queue_size=10)
        self.pub_detafr=rospy.Publisher("/detafr",Float64,queue_size=10)
        self.pub_detarl=rospy.Publisher("/detarl",Float64,queue_size=10)
        self.pub_detarr=rospy.Publisher("/detarr",Float64,queue_size=10)
        self.pub_detafl_initial=rospy.Publisher("/detafli",Float64,queue_size=10)
        self.pub_detafr_initial=rospy.Publisher("/detafri",Float64,queue_size=10)
        self.pub_rotation_linear_velocity=rospy.Publisher("/rotation_linear_velocity",Float64,queue_size=10)
        self.pub_rotation_acumulation_error=rospy.Publisher("/rotation_acumulation_error",Float64,queue_size=10)
        # Vfl,Vfr,Vrl,Vrr,detafl,detafr,detarl,detarr
        self.target_path=[]
        self.homing_original_position=[self.mpfh.Driver_steer_encode_fl_original,self.mpfh.Driver_steer_encode_fr_original,self.mpfh.Driver_steer_encode_rl_original,self.mpfh.Driver_steer_encode_rr_original]
    def CmdVel_callback(self,msg):
        """
        Read linear velocity from topic cmd_vel
        """
        # print "msg",msg.linear.x
        self.linear_x=msg.linear.x
        self.linear_y=msg.linear.y
        self.linear_z=msg.linear.z
        self.angular_x=msg.angular.x
        self.angular_y=msg.angular.y
        self.angular_z=msg.angular.z
    def PathTarget_callback(self,msg):
        for i in range(len(msg.poses)):
            self.pose_x=msg.poses[i].pose.position.x
            self.pose_y=msg.poses[i].pose.position.y
            self.pose_z=msg.poses[i].pose.position.z
            self.pose_quaternion_x=msg.poses[i].pose.orientation.x
            self.pose_quaternion_y=msg.poses[i].pose.orientation.y
            self.pose_quaternion_z=msg.poses[i].pose.orientation.z
            self.pose_quaternion_w=msg.poses[i].pose.orientation.w
            orientation_list = [self.pose_quaternion_x, self.pose_quaternion_y, self.pose_quaternion_z,self.pose_quaternion_w]
            (self.roll,self.pitch,self.yaw) = euler_from_quaternion (orientation_list)
            self.path_all.append([self.pose_x,self.pose_y,self.yaw])
            # print("-------path---data----",[self.pose_x,self.pose_y,self.pose_z,self.roll,self.pitch,self.yaw])
    def Avage_list(self,listdata,appendata):
        if len(listdata)>10:
            listdata=listdata[1:]
            listdata.append(appendata)
        else:
            listdata.append(appendata)
        return listdata
    def Init_Node(self):
        self.mpfh.Init_can()
        self.mpfh.Open_driver_can_Node(0x00000000,1)
        self.mpfh.Clear_all_error_without_disable_driver()
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
        print("----d------",d)
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
            return [-1.0*self.vel_reference,1.0*self.vel_reference,-1.0*self.vel_reference,1.0*self.vel_reference]
            # return [-0.50*self.vel_reference,0.50*self.vel_reference,-0.50*self.vel_reference,0.50*self.vel_reference]
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
        """
        Nomalization in [-pi,pi]
        """
        return (rad+pi)%(pi*2.0) - pi
    def sign(self,x):
        if x!=0.0:
            return x/abs(x)
        else:
            return 0.0
    def caculate_pos_negtive_phi(self,target,realpoint):
        if abs(target-realpoint)>pi:
            if target<0:
                return target-realpoint+2*pi
            elif target>0:
                return target+realpoint+2*pi
            else:
                return target-realpoint
        else:
            return target-realpoint
    def hoffman_control_tf2(self,point_ref_all,odemetry_x,odemetry_y,odemetry_pha):
        self.odemetry_x=odemetry_x#self.odemetry_x+self.vel_reference*cos(self.odemetry_pha)*dt
        self.odemetry_y=odemetry_y#self.odemetry_y+self.vel_reference*sin(self.odemetry_pha)*dt
        self.odemetry_pha=odemetry_pha#self.odemetry_pha+self.vel_reference*(1.0/self.car_length)*self.st*dt
        self.odemetry_pha=self.Set_rad_in_pi(self.odemetry_pha)
        e=sqrt((self.odemetry_x-point_ref_all[0][0])**2+(self.odemetry_y-point_ref_all[0][1])**2)
        for i in range(1,len(point_ref_all)):
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
        self.pub_angular_e.publish(e)
        self.phi=point_ref[2]- self.odemetry_pha+atan(self.kk*e/self.vel_reference)
        self.st=tan(self.phi)
        if abs(tan(self.st))>=self.limit_steer_rad:
            self.st=self.limit_steer_rad*self.sign(self.st)
        else:
            self.st=self.st
        self.phi=atan(self.st)
        self.pub_angular_error.publish(self.phi)

    def target_distance_error(self,limit_error_trans,x,y):
        e=sqrt((self.path_all[-1][0]-x)**2+(self.path_all[-1][1]-y)**2)
        self.pub_error.publish(e)
        if e<limit_error_trans:
            print("distance in line error----")
            return True
        else:
            return False
    def target_distance_error_data(self,x,y):
        e=sqrt((self.path_all[-1][0]-x)**2+(self.path_all[-1][1]-y)**2)
        return e
    def traget_pha_error(self,real_pha,target_pha):
        return -real_pha+target_pha
    def set_array_to_list(self,array_num):
        newtemp=[]
        for i in range(len(array_num)):
            newtemp.append(list(array_num[i]))
        return newtemp
    def hoffman_kinematic_model_new(self,VC,phi_ref):
        Vfl=VC*sqrt((0.5*self.car_length)**2+(0.5*self.car_length/tan(phi_ref)-self.car_width/2.0)**2)/(0.5*self.car_length/tan(phi_ref))
        Vfr=VC*sqrt((0.5*self.car_length)**2+(0.5*self.car_length/tan(phi_ref)+self.car_width/2.0)**2)/(0.5*self.car_length/tan(phi_ref))#VC*sqrt(self.car_length**2+(self.car_length/tan(phi_ref)-self.car_width/2.0)**2)/(self.car_length/tan(phi_ref))
        Vrl=Vfl#VC*(self.car_length/tan(phi_ref)+self.car_width/2.0)/(self.car_length/tan(phi_ref))
        Vrr=Vfr#VC*(self.car_length/tan(phi_ref)-self.car_width/2.0)/(self.car_length/tan(phi_ref))

        detafl=atan(0.5*self.car_length/(0.5*self.car_length/tan(phi_ref)-self.car_width/2.0))
        detafr=atan(0.5*self.car_length/(0.5*self.car_length/tan(phi_ref)+self.car_width/2.0))
        self.pub_detafl_initial.publish(detafl)
        self.pub_detafr_initial.publish(detafr)
        limit_degree=30.0
        if abs(detafl)>(limit_degree*pi/180):
            if detafl>0.0:
                detafl=limit_degree*pi/180
            elif detafl<0.0:
                detafl=-limit_degree*pi/180
            else:
                detafl=0.0
        else:
            detafl=detafl
        if abs(detafr)>(limit_degree*pi/180):
            if detafr>0.0:
                detafr=limit_degree*pi/180
            elif detafr<0.0:
                detafr=-limit_degree*pi/180
            else:
                detafr=0.0
        else:
            detafr=detafr
        detarl=-detafl
        detarr=-detafr
        self.pub_Vfl.publish(Vfl)
        self.pub_Vfr.publish(Vfr)
        self.pub_Vrl.publish(Vrl)
        self.pub_Vrr.publish(Vrr)
        self.pub_detafl.publish(detafl)
        self.pub_detafr.publish(detafr)
        self.pub_detarl.publish(detarl)
        self.pub_detarr.publish(detarr)
        return [Vfl,Vfr,Vrl,Vrr,detafl,detafr,detarl,detarr]
    def caculate_target_to_real_angular(self,target_xy_pha,real_xy_pha):
        """
        target_xy_pha-->[x,y]
        """
        if target_xy_pha[0]-real_xy_pha[0]!=0 and target_xy_pha[1]-real_xy_pha[1]!=0:
            return atan((target_xy_pha[1]-real_xy_pha[1])/(target_xy_pha[0]-real_xy_pha[0]))-real_xy_pha[2]
        elif target_xy_pha[0]-real_xy_pha[0]==0 and target_xy_pha[1]-real_xy_pha[1]!=0:
            if target_xy_pha[1]-real_xy_pha[1]>0 and self.sign(target_xy_pha[2])==self.sign(real_xy_pha[2]):#pha have same symbol
                return pi/2-real_xy_pha[2]
            elif target_xy_pha[1]-real_xy_pha[1]<0 and self.sign(target_xy_pha[2])==self.sign(real_xy_pha[2]):
                return -pi/2-real_xy_pha[2]
            elif target_xy_pha[1]-real_xy_pha[1]>0 and self.sign(target_xy_pha[2])!=self.sign(real_xy_pha[2]):
                return -pi/2-real_xy_pha[2]
            else:# target_xy_pha[1]-real_xy_pha[1]<0 and self.sign(target_xy_pha[2])!=self.sign(real_xy_pha[2]):
                return pi/2-real_xy_pha[2]
        elif target_xy_pha[0]-real_xy_pha[0]!=0 and target_xy_pha[1]-real_xy_pha[1]==0:
            if target_xy_pha[0]-real_xy_pha[0]>0 and self.sign(target_xy_pha[2])==self.sign(real_xy_pha[2]):#pha have same symbol
                return 0.0-real_xy_pha[2]
            elif target_xy_pha[0]-real_xy_pha[0]<0 and self.sign(target_xy_pha[2])==self.sign(real_xy_pha[2]):
                return pi-real_xy_pha[2]
            elif target_xy_pha[0]-real_xy_pha[0]>0 and self.sign(target_xy_pha[2])!=self.sign(real_xy_pha[2]):
                return 0.0-real_xy_pha[2]
            else:# target_xy_pha[1]-real_xy_pha[1]<0 and self.sign(target_xy_pha[2])!=self.sign(real_xy_pha[2]):
                return pi-real_xy_pha[2]
        else:
            return 0.0-real_xy_pha[2]

def main():
   agvobj=AGV4WDICONTROLLER()
   agvobj.Init_Node()
   time.sleep(3)
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
   speedfl=0
   speedfr=0
   speedrl=0
   speedrr=0
   flag_for_servo_first_step=0
   flag_go_x_shape=0
   flagg=1
   flaggg=1
   pathfilename='path'#'pathsmallCirclexythera'
   limit_error_rad_for_start_point=0.1#rad
   rotation_velocity=0.5
   flag_path_tracking=0
   flag_send_ref_velocity_for_onece=0
   flag_close_all=1
   limit_error_trans=0.5
   flag_for_servo_to_end_point=0
   limit_error_distance=0.15
   limit_error_for_pha=0.4
   flag_open_servo_pha=0.0
   flag_open_pi_4=0.0
   first_step_roation_linear_velocity=0
   first_step_roation_kp=1/(2*pi)
   first_step_roation_ki=0.1/(2*pi)
   first_step_roation_error=0
   first_step_roation_error_max=10.0
   while not rospy.is_shutdown():
        if len(agvobj.path_all)!=0:
            try:
                trans = agvobj.tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("tf2 have nothing-----")
                rate.sleep()
                continue
            recevenum=agvobj.mpfh.CanAnalysis.Can_GetReceiveNum(0)
            starttime=time.time()
            print("recevenum----",recevenum)
            if recevenum!=None and flag_close_all==1:
                agvobj.mpfh.Read_sensor_data_from_driver()
                odemetry_xx=trans.transform.translation.x
                odemetry_yy=trans.transform.translation.y
                print("odemetry_xx,odemetry_yy",odemetry_xx,odemetry_yy)
                ratation_quaternion=[trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w]
                rr,pp,yy=euler_from_quaternion(ratation_quaternion)
                print("mobile real -----rpy----",rr,pp,yy)
                odemetry_pha_pha=yy
                agvobj.pub_x.publish(odemetry_xx)
                agvobj.pub_y.publish(odemetry_yy)
                agvobj.pub_real_pha.publish(odemetry_pha_pha)
                agvobj.pub_pha_error.publish(agvobj.traget_pha_error(agvobj.path_all[0][2],odemetry_pha_pha))
                first_step_roation_error+=agvobj.traget_pha_error(agvobj.path_all[0][2],odemetry_pha_pha)*dt
                if first_step_roation_error>=first_step_roation_error_max:
                    first_step_roation_error=0
                first_step_roation_linear_velocity=first_step_roation_kp*abs(agvobj.traget_pha_error(agvobj.path_all[0][2],odemetry_pha_pha))+first_step_roation_ki*first_step_roation_error
                agvobj.pub_rotation_acumulation_error.publish(first_step_roation_error)
                # first_step_roation_linear_velocity=first_step_roation_k*abs(agvobj.traget_pha_error(agvobj.path_all[0][2],odemetry_pha_pha))
                agvobj.pub_rotation_linear_velocity.publish(first_step_roation_linear_velocity)
                print("agvobj.path_all[0][2],odemetry_pha_pha,error",agvobj.path_all[0][2],odemetry_pha_pha,agvobj.traget_pha_error(agvobj.path_all[0][2],odemetry_pha_pha))
                if flag_for_servo_first_step==0 and abs(agvobj.traget_pha_error(agvobj.path_all[0][2],odemetry_pha_pha))>limit_error_rad_for_start_point:
                    rospy.loginfo("------First: Servo to the latest pha error with mobile coordinate------")
                   
                    agvobj.pub_pha_error.publish(agvobj.traget_pha_error(agvobj.path_all[0][2],odemetry_pha_pha))
                    if flag_go_x_shape==0:
                        agvobj.mpfh.Send_same_degree_position_to_four_steering_wheel([1.0,-1.0,-1.0,1.0],pi/4)
                        time.sleep(1)
                        flag_go_x_shape=1
                    if agvobj.traget_pha_error(agvobj.path_all[0][2],odemetry_pha_pha)>0.0:
                        print("agvobj.traget_pha_error(agvobj.path_all[0][2],odemetry_pha_pha)",agvobj.traget_pha_error(agvobj.path_all[0][2],odemetry_pha_pha))
                        agvobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1.0,-1.0,-1.0,-1.0],1.0,first_step_roation_linear_velocity)#clockwise
                    else:
                        agvobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1.0,-1.0,-1.0,-1.0],-1.0,first_step_roation_linear_velocity)#anticlockwise#change the speed flag to positive we need -1.0
                elif flag_for_servo_first_step==0 and abs(agvobj.traget_pha_error(agvobj.path_all[0][2],odemetry_pha_pha))<=limit_error_rad_for_start_point:
                    agvobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1.0,1.0,-1.0,1.0],1,0.0)
                    agvobj.mpfh.Send_diff_degree_position_to_four_steering_wheel([-1.0,-1.0,-1.0,-1.0],[0.0,0.0,0.0,0.0])
                    time.sleep(2)
                    flag_for_servo_first_step=1
                    flag_path_tracking=0
                    flag_send_ref_velocity_for_onece=0
                else:
                    pass
                if flag_send_ref_velocity_for_onece:
                    rospy.loginfo("------: send initial velocity------")
                    agvobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1.0,1.0,-1.0,1.0],1,agvobj.vel_reference)
                    flag_send_ref_velocity_for_onece=0
                if flag_path_tracking:
                    rospy.loginfo("------Second: Path tracking Start&Ongoing------")
                    agvobj.hoffman_control_tf2(agvobj.path_all,odemetry_xx,odemetry_yy,odemetry_pha_pha)
                    if agvobj.target_distance_error(limit_error_trans,odemetry_xx,odemetry_yy):
                        rospy.loginfo("------Second: Path tracking End------")
                        agvobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1.0,1.0,-1.0,1.0],1,0.0)
                        agvobj.mpfh.Send_same_degree_position_to_four_steering_wheel([-1.0,-1.0,-1.0,-1.0],0.0)
                        time.sleep(2)
                        # time.sleep(1)
                        flag_path_tracking=0
                        flag_for_servo_to_end_point=1
                        # flag_close_all=0
                    velocity_real_time=agvobj.Caculate_velocity_from_RPM()
                    if len(velocity_real_time)!=0 and flag_path_tracking==1:
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
                        # VC=(agvobj.sign(Vri)*Vri+agvobj.sign(Vro)*Vro)/2.
                        VC=agvobj.vel_reference
                        print("VC-------",VC)
                        v_deta=agvobj.hoffman_kinematic_model_new(VC,agvobj.phi)
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


                if flag_for_servo_to_end_point:
                    rospy.loginfo("------Third: Servo to end point with ------")
                    # rospy.loginfo("distance error----",agvobj.target_distance_error_data(odemetry_xx,odemetry_yy))
                    print("abs(agvobj.target_distance_error_data(odemetry_xx,odemetry_yy))",abs(agvobj.target_distance_error_data(odemetry_xx,odemetry_yy)))
                    agvobj.pub_error.publish(agvobj.target_distance_error_data(odemetry_xx,odemetry_yy))
                    if abs(agvobj.target_distance_error_data(odemetry_xx,odemetry_yy))>=limit_error_distance:
                        phi_to_endpoint=agvobj.caculate_target_to_real_angular([agvobj.path_all[-1][0],agvobj.path_all[-1][1],agvobj.path_all[-1][2]],[agvobj.odemetry_x,agvobj.odemetry_y,agvobj.odemetry_pha])
                        agvobj.mpfh.Send_same_degree_position_to_four_steering_wheel([-1.0,-1.0,-1.0,-1.0],phi_to_endpoint)
                        agvobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1.0,1.0,-1.0,1.0],1,rotation_velocity)
                    else:
                        print("------------------in------third-------status---------")
                        agvobj.mpfh.Send_same_degree_position_to_four_steering_wheel([-1.0,-1.0,-1.0,-1.0],0.0)
                        agvobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1.0,1.0,-1.0,1.0],1,0.0)
                        agvobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1.0,1.0,-1.0,1.0],1,0.0)
                        time.sleep(1.5)
                        flag_open_servo_pha=1
                        flag_for_servo_to_end_point=0
                if flag_open_servo_pha:
                    print("abs(agvobj.path_all[-1][2]-agvobj.odemetry_pha)",abs(agvobj.path_all[-1][2]-agvobj.odemetry_pha))
                    agvobj.pub_pha_error.publish(agvobj.path_all[-1][2]-agvobj.odemetry_pha)
                    if abs(agvobj.path_all[-1][2]-agvobj.odemetry_pha)>=limit_error_for_pha:
                        if flag_open_pi_4==0:
                            agvobj.mpfh.Send_same_degree_position_to_four_steering_wheel([1.0,-1.0,-1.0,1.0],pi/4)
                            time.sleep(2)
                            flag_open_pi_4=1
                        if agvobj.path_all[-1][2]-agvobj.odemetry_pha>0.0 and agvobj.sign(agvobj.path_all[-1][2])==agvobj.sign(agvobj.odemetry_pha):
                            agvobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1.0,-1.0,-1.0,-1.0],1.0,rotation_velocity)
                        elif agvobj.path_all[-1][2]-agvobj.odemetry_pha<0.0 and agvobj.sign(agvobj.path_all[-1][2])==agvobj.sign(agvobj.odemetry_pha):
                            agvobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1.0,-1.0,-1.0,-1.0],-1.0,rotation_velocity)
                        elif agvobj.path_all[-1][2]-agvobj.odemetry_pha>0.0 and agvobj.sign(agvobj.path_all[-1][2])!=agvobj.sign(agvobj.odemetry_pha):
                            agvobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1.0,-1.0,-1.0,-1.0],-1.0,rotation_velocity)
                        else:
                            agvobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1.0,-1.0,-1.0,-1.0],1.0,rotation_velocity)
                    else:
                        flag_open_servo_pha=0
                        agvobj.mpfh.Send_same_degree_position_to_four_steering_wheel([-1.0,-1.0,-1.0,-1.0],0.0)
                        agvobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1.0,1.0,-1.0,1.0],1,0.0)
                        time.sleep(2)
                        flag_close_all=0
                count+=1
                print("count",count)
            else:
                if flag_close_all==0:
                    agvobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1.0,1.0,-1.0,1.0],1,0.0)
                    agvobj.mpfh.Send_diff_degree_position_to_four_steering_wheel(ratation_flag,[0.0,0.0,0.0,0.0])
                agvobj.mpfh.Send_Control_Command(agvobj.mpfh.CanAnalysis.yamlDic['sync_data_ID'], agvobj.mpfh.MobileDriver_Command.ZERO_COMMAND)
                print("---------read data----")
            endtime=time.time()
            dt=endtime-starttime
            print("-----------dt------",dt)
            # agvobj.
        else:
            print("----wait path----",agvobj.path_all)
            # continue

        rate.sleep() 
   agvobj.mpfh.CanAnalysis.Can_VCICloseDevice()
   agvobj.mpfh.Close_driver_can_Node(0x00000000,1)
if __name__=="__main__":
    main()