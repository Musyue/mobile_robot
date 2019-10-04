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
        # self.mpfh=MobilePlatformDriver()
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
        self.pub_vstar=rospy.Publisher("/vstar",Float64,queue_size=10)
        self.pub_x=rospy.Publisher("/x",Float64,queue_size=10)
        self.pub_y=rospy.Publisher("/y",Float64,queue_size=10)
        self.pub_theta=rospy.Publisher("/theta",Float64,queue_size=10)
        self.k_rou=0.2#3#0.2
        self.k_alpha=0.5#8#0.5
        self.k_beta=-0.2#-3#-0.2
        # self.homing_original_position=[self.mpfh.Driver_steer_encode_fl_original,self.mpfh.Driver_steer_encode_fr_original,self.mpfh.Driver_steer_encode_rl_original,self.mpfh.Driver_steer_encode_rr_original]
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
        # self.mpfh.Init_can()
        # self.mpfh.Open_driver_can_Node(0x00000000,1)
        # self.mpfh.Enable_Motor_Controller_All()
        # self.mpfh.Send_trapezoid_Velocity(2500)
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
    def Control_to_pose(self,dt,target):
        # print "haha"
        delta_x=self.odemetry_x-target[0]
        delta_y=self.odemetry_y-target[1]
        rou=sqrt(delta_x**2+delta_y**2)
        alpha=atan(delta_y/delta_x)-self.odemetry_theta
        beta=-self.odemetry_theta-alpha+target[2]
        delta_rou=-self.k_rou*cos(alpha)
        delta_alpha=self.k_rou*sin(alpha)-self.k_alpha*alpha-self.k_beta*beta
        delta_beta=-self.k_rou*sin(alpha)
        rou=rou+delta_rou*dt
        alpha=alpha+delta_alpha*dt
        beta=beta+delta_beta*dt
        vel=self.k_rou*rou
        gamma=self.k_alpha*alpha+self.k_beta*beta
        
        delta_theta=(tan(gamma)*vel)/(self.car_length)*dt
        delta_x=vel*cos(self.odemetry_theta)*dt
        delta_y=vel*sin(self.odemetry_theta)*dt
        # print 
        self.odemetry_x=self.odemetry_x+delta_x
        self.odemetry_y=self.odemetry_y+delta_y
        self.odemetry_theta=self.odemetry_theta+delta_theta
        self.pub_vstar.publish(vel)
        self.pub_x.publish(self.odemetry_x)
        self.pub_y.publish(self.odemetry_y)
        self.pub_theta.publish(self.odemetry_theta)
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
        print "th1,th2",th1,th2
        return (d+pi)%(2.0*pi) - pi
    # def mod_function(self,a,m):
    #     return a - m*int(a/m)

    def Caculate_velocity_from_angular_z(self,angular_velocity_z,gamma_rad):
        vel=(angular_velocity_z*self.car_length)/tan(gamma_rad)
        return vel
    # def Caculate_velocity_from_RPM(self):
    #     # Velocity=[]
    #     if self.mpfh.Driver_walk_velocity_encode_fl!=0 and self.mpfh.Driver_walk_velocity_encode_fr!=0 and self.mpfh.Driver_walk_velocity_encode_rl!=0 and self.mpfh.Driver_walk_velocity_encode_rr!=0:
    #         # print self.mpfh.Driver_walk_velocity_encode_fl
    #         # print self.mpfh.Driver_walk_velocity_encode_fr
    #         # print self.mpfh.Driver_walk_velocity_encode_rl
    #         # print self.mpfh.Driver_walk_velocity_encode_rr
    #         RPM_fl=self.mpfh.Dec_to_RPM(self.mpfh.Driver_walk_velocity_encode_fl)
    #         RPM_fr=self.mpfh.Dec_to_RPM(self.mpfh.Driver_walk_velocity_encode_fr)
    #         RPM_rl=self.mpfh.Dec_to_RPM(self.mpfh.Driver_walk_velocity_encode_rl)
    #         RPM_rr=self.mpfh.Dec_to_RPM(self.mpfh.Driver_walk_velocity_encode_rr)
    #         Velocity=[(RPM_fl*2*pi*self.wheel_R)/60.0,(RPM_fr*2*pi*self.wheel_R)/60.0,(RPM_rl*2*pi*self.wheel_R)/60.0,(RPM_rr*2*pi*self.wheel_R)/60.0]
    #         print "------Velocity---------",Velocity#self.mpfh.Driver_walk_velocity_encode_fl
    #         if max(Velocity)-min(Velocity)>10.0:
    #             return 0.0
    #         else:
    #             print "From Wheel Encode Velocity-------",(abs(Velocity[0])+abs(Velocity[1])+abs(Velocity[2])+abs(Velocity[3]))/4
    #             return (abs(Velocity[0])+abs(Velocity[1])+abs(Velocity[2])+abs(Velocity[3]))/4
    #     else:
    #         return 0.0
    def caculate_bicycle_model_thetafr_re(self):

        # print self.linear_x,self.angular_z
        if self.linear_x!=0:
            thetafr=atan((self.angular_z*self.car_length)/(2.0*self.linear_x))
            thetare=atan(-tan(thetafr))
            return [thetafr,thetare]
        else:
            return [0.0,0.0]
    def my_arccot(self,x):
        return pi/2-atan(x)
        # if x>0:
        #     return atan(1/x)+pi
        # elif x<0:
        #     return atan(1/x)
        # else:
        #     return 0.0

    def caculate_four_steer_degree_theta(self):
        """
        arccot(x)=
        {
            arctan(1/x)+π(x>0)
            arctan(1/x)(x<0)
        }
        """
        temp_fr_re=self.caculate_bicycle_model_thetafr_re()
        # numpy.arccot()
        if 0 not in temp_fr_re:
            temp_theta_fo_fr=(1/tan(temp_fr_re[0]))*(1+(self.car_width/self.car_length)*(tan(temp_fr_re[0])-tan(temp_fr_re[1])))
            temp_theta_fi_fl=(1/tan(temp_fr_re[0]))*(1-(self.car_width/self.car_length)*(tan(temp_fr_re[0])-tan(temp_fr_re[1])))
            temp_theta_ro_rr=(1/tan(temp_fr_re[1]))*(1+(self.car_width/self.car_length)*(tan(temp_fr_re[0])-tan(temp_fr_re[1])))
            temp_theta_ri_rl=(1/tan(temp_fr_re[1]))*(1-(self.car_width/self.car_length)*(tan(temp_fr_re[0])-tan(temp_fr_re[1])))
            theta_fo_fr=self.my_arccot(temp_theta_fo_fr)
            theta_fi_fl=self.my_arccot(temp_theta_fi_fl)
            theta_ro_rr=self.my_arccot(temp_theta_ro_rr)
            theta_ri_rl=self.my_arccot(temp_theta_ri_rl)
            return [theta_fo_fr,temp_theta_fi_fl,temp_theta_ro_rr,temp_theta_ri_rl]
        else:
            # print "bicycle model temp_fr_re data may be not right------",temp_fr_re
            return [0.0,0.0,0.0,0.0]
    def caculate_four_walk_motor_velocity(self):
        temp_theta_fl_fr_rl_rr=self.caculate_four_steer_degree_theta()
        temp_fr_re=self.caculate_bicycle_model_thetafr_re()
        if 0.0 not in temp_fr_re:
            v1_fr_fo=(self.linear_x*tan(temp_fr_re[0]*(1/sin(temp_theta_fl_fr_rl_rr[0]))))/sqrt(1+(1/4)*(tan(temp_fr_re[0])-tan(temp_fr_re[1]))**2)
            v2_fl_fi=(self.linear_x*tan(temp_fr_re[0]*(1/sin(temp_theta_fl_fr_rl_rr[1]))))/sqrt(1+(1/4)*(tan(temp_fr_re[0])-tan(temp_fr_re[1]))**2)
            v3_rr_ro=(self.linear_x*tan(temp_fr_re[0]*(1/sin(temp_theta_fl_fr_rl_rr[2]))))/sqrt(1+(1/4)*(tan(temp_fr_re[0])-tan(temp_fr_re[1]))**2)
            v4_rl_ri=(self.linear_x*tan(temp_fr_re[0]*(1/sin(temp_theta_fl_fr_rl_rr[3]))))/sqrt(1+(1/4)*(tan(temp_fr_re[0])-tan(temp_fr_re[1]))**2)
            return [v1_fr_fo,v2_fl_fi,v3_rr_ro,v4_rl_ri] 
        else:
            # print "the mobile platform walking in line---"  
            return [self.linear_x,self.linear_x,self.linear_x,self.linear_x] 
    def Control_mobile_to_one_target(self,x,y,theta,Kv,Kh,dt):
        """
        World coordinate:[x,y,theta]
        """

        Vstar=Kv*sqrt((x-self.odemetry_x)**2+(y-self.odemetry_y)**2)
        # print "self.odemetry_x",self.odemetry_x
        print "x-self.odemetry_x,y-self.odemetry_y",x-self.odemetry_x,y-self.odemetry_y
        thetastar=atan((y-self.odemetry_y)/(x-self.odemetry_x))
        gammastar=Kh*self.andiff(thetastar,self.odemetry_theta)#Kh>0
        print "thetastar,theta",thetastar,theta
        self.Bycycle_Model(Vstar,gammastar,dt)
        print "------gammastar-----",gammastar,Vstar,thetastar
        self.pub_vstar.publish(Vstar)
        self.pub_x.publish(self.odemetry_x)
        self.pub_y.publish(self.odemetry_y)
        self.pub_theta.publish(self.odemetry_theta)
        # self.mpfh.Send_same_velocity_to_four_walking_wheel([-1,1,-1,1],1,Vstar)
        # self.mpfh.Send_same_degree_position_to_four_steering_wheel([1.0,-1.0,-0.0,0.0],gammastar)
    # def Cmd_vel_control_for_test(self):

    #     if self.linear_x!=0 or self.angular_z!=0:
    #         if self.linear_x!=0:
    #             #print self.mpfh.Driver_steer_encode_fl,abs(self.mpfh.Driver_steer_encode_fl-self.mpfh.Driver_steer_encode_fl_original)
    #             if abs(self.mpfh.Driver_steer_encode_fl-self.mpfh.Driver_steer_encode_fl_original)>100 or abs(self.mpfh.Driver_steer_encode_fr-self.mpfh.Driver_steer_encode_fr_original)>100 or abs(self.mpfh.Driver_steer_encode_rl-self.mpfh.Driver_steer_encode_rl_original)>100 or abs(self.mpfh.Driver_steer_encode_rr-self.mpfh.Driver_steer_encode_rr_original)>100: 
    #                 self.mpfh.Send_position_to_four_steering_wheel(self.homing_original_position)
    #             if self.linear_x==self.linear_x:
    #                 self.mpfh.Send_same_velocity_to_four_walking_wheel([-1,1,-1,1],1,self.linear_x)
    #                 time.sleep(0.03)
    #         if self.angular_z!=0:
    
    #             self.mpfh.Send_same_degree_position_to_four_steering_wheel([1.0,-1.0,-1.0,1.0],pi/4)
    #             speed=self.Caculate_velocity_from_angular_z(self.angular_z,pi/4)
    #             print "speed",speed
    #             if self.angular_z==self.angular_z and abs(speed)>0.0001:
    #                 self.mpfh.Send_same_velocity_to_four_walking_wheel([-1.0,-1.0,-1.0,-1.0],1,speed)
    #     else:
    #         # print "Velocity is Zero"
    #         time.sleep(0.00015)
    #         self.mpfh.Send_same_velocity_to_four_walking_wheel([-1,1,-1,1],1,0)

def main():
   imuobj=IMUDATAINTERGAL()
   imuobj.Init_Node()
   ratet=10
   rate=rospy.Rate(ratet)
   zerotime=time.time()
#    theta=0#(x,y,theta)
   gama=0#(roation angular)
   v=0.1331
   dt=0
   x=0
   y=0
   flg=0
   xg=[10,5,pi/2]
   x0=[2,5,-pi/4]
   imuobj.set_pdemetry_x(x0[0])
   imuobj.set_pdemetry_y(x0[1])
   imuobj.set_pdemetry_theta(x0[2])
   dtt=0
#    dttt=0
#    dtttt=0
   while not rospy.is_shutdown():
        # recevenum=imuobj.mpfh.CanAnalysis.Can_GetReceiveNum(0)
        # starttime=time.time()
        now = rospy.get_rostime()
        startros=now.nsecs/10.0**9
        rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
        # print "recevenum",recevenum
        # if recevenum!=None:
            # if flg==0:
            #     imuobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1,1,-1,1],1,1.0)
            #     flg=1
            # print imuobj.linear_x
            # imuobj.mpfh.Read_sensor_data_from_driver()
            # imuobj.mpfh.Send_same_velocity_to_four_walking_wheel([-1.0,-1.0,-1.0,-1.0],1,0.5)
            # kk=imuobj.Cmd_vel_control_for_test()

            # Vel=imuobj.Caculate_velocity_from_RPM()
            # imuobj.Bycycle_Model(Vel,0.0)
        if dt!=0:
            print "----dt---",dt
            dtt+=dt
            # dtttt+=dttt
        imuobj.Control_to_pose(dt,xg)
            # imuobj.Control_mobile_to_one_target(xg[0],xg[1],0.0,0.5,4,0.1)
        # else:
            # imuobj.mpfh.Send_Control_Command( imuobj.mpfh.CanAnalysis.yamlDic['sync_data_ID'], imuobj.mpfh.MobileDriver_Command.ZERO_COMMAND)
        # endtime=time.time()
        end=rospy.get_rostime()
        endros=end.nsecs/10.0**9
        dt=endros-startros
        # dt=endtime-starttime

        print "dtt",dtt
        rate.sleep() 
if __name__=="__main__":
    main()