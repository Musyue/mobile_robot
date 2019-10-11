#! /usr/bin/env python
# coding=utf-8
import rospy
import sys
from std_msgs.msg import String,Float64,Bool,Int64MultiArray
from sensor_msgs.msg import Imu
import time 
from math import *
import numpy as np

from scipy.io import loadmat
import matplotlib.pyplot as plt
class AGV4WDIPLOT():
    def __init__(self):
        self.xr=[]
        self.yr=[]
        self.target_pha=[]
        self.x=[]
        self.y=[]
        self.real_pha=[]
        self.Vfl=[]
        self.Vfr=[]
        self.Vrl=[]
        self.Vrr=[]
        self.detafl=[]
        self.detafr=[]
        self.error=[]
        self.sub_xr=rospy.Subscriber("/xr",Float64,self.xr_callback)
        self.sub_yr=rospy.Subscriber("/yr",Float64,self.yr_callback)
        self.sub_target_pha=rospy.Subscriber("/target_pha",Float64,self.target_pha_callback)
        self.sub_x=rospy.Subscriber("/x",Float64,self.x_callback)
        self.sub_y=rospy.Subscriber("/y",Float64,self.x_callback)
        self.sub_real_pha=rospy.Subscriber("/real_pha",Float64,self.real_pha_callback)
        self.sub_Vfl=rospy.Subscriber("/vfl",Float64,self.vfl_callback)
        self.sub_Vfr=rospy.Subscriber("/vfr",Float64,self.vfr_callback)
        self.sub_Vrl=rospy.Subscriber("/vrl",Float64,self.vrl_callback)
        self.sub_Vrr=rospy.Subscriber("/vrr",Float64,self.vrr_callback)
        self.sub_detafl=rospy.Subscriber("/detafl",Float64,self.detafl_callback)
        self.sub_detafr=rospy.Subscriber("/detafr",Float64,self.detafr_callback)
        self.sub_error=rospy.Subscriber("/distance_error",Float64,self.error_callback)
    def error_callback(self,msg):
        self.Avage_list(self.error,msg.data)
        # self.xrmsg.data
    def xr_callback(self,msg):
        self.Avage_list(self.xr,msg.data)
        # self.xrmsg.data
    def yr_callback(self,msg):
        self.Avage_list(self.yr,msg.data)
    def target_pha_callback(self,msg):
        self.Avage_list(self.target_pha,msg.data)
    def x_callback(self,msg):
        self.Avage_list(self.x,msg.data)
    def y_callback(self,msg):
        self.Avage_list(self.y,msg.data)
    def real_pha_callback(self,msg):
        self.Avage_list(self.real_pha,msg.data)

    def vfl_callback(self,msg):
        self.Avage_list(self.Vfl,msg.data)
    def vfr_callback(self,msg):
        self.Avage_list(self.Vfr,msg.data)
    def vrl_callback(self,msg):
        self.Avage_list(self.Vrl,msg.data)
    def vrr_callback(self,msg):
        self.Avage_list(self.Vrr,msg.data)
    def detafl_callback(self,msg):
        self.Avage_list(self.detafl,msg.data)
    def detafr_callback(self,msg):
       self.Avage_list(self.detafr,msg.data)        
    def Avage_list(self,listdata,appendata):
        if len(listdata)>10:
            listdata=listdata[1:]
            listdata.append(appendata)
        else:
            listdata.append(appendata)
        return listdata
    def Init_Node(self):
        rospy.init_node("plot_data_for_mobileplatform")

    
def main():
    agvplot=AGV4WDIPLOT()
    agvplot.Init_Node()
    ratet=10
    rate=rospy.Rate(ratet)
    zerotime=time.time()
    dt=0

    flg=0
    count=1
    xr=[]
    yr=[]
    x=[]
    y=[]
    plt.ion() #开启interactive mode 成功的关键函数
    fig1=plt.figure(1)
    ax0 = fig1.add_subplot(1, 1, 1)
    fig2=plt.figure(2)
    fig3=plt.figure(3)
    vfr=[]
    vfl=[]
    vrl=[]
    vrr=[]
    ax1 = fig2.add_subplot(2, 2, 1)
    ax2 = fig2.add_subplot(2, 2, 2)
    ax3 = fig2.add_subplot(2, 2, 3)
    ax4 = fig2.add_subplot(2, 2, 4)
    detafl=[]
    detafr=[]
    ax5 = fig3.add_subplot(2, 2, 1)
    ax6 = fig3.add_subplot(2, 2, 2)
    ax7 = fig3.add_subplot(2, 2, 3)
    plt.show()
    while not rospy.is_shutdown():

        if len(agvplot.xr)!=0:
            try:
                xr.append(agvplot.xr[-1])
                yr.append(agvplot.yr[-1])
                x.append(agvplot.x[-1])
                y.append(agvplot.y[-1])
                ax0.plot(xr,yr,'ro',x,y,'bs')
                vfl.append(agvplot.Vfl[-1])
                vfr.append(agvplot.Vfr[-1])
                vrl.append(agvplot.Vrl[-1])
                vrr.append(agvplot.Vrr[-1])
                ax1.plot(vfl)
                ax2.plot(vfr)
                ax3.plot(vrl)
                ax4.plot(vrr)
                ax5.plot(detafl)
                ax6.plot(detafr)
                plt.draw()
                plt.pause(0.01)
                count+=1
                print("count",count)
            except:
                print("haha")

        rate.sleep() 

if __name__=="__main__":
    main()