from math import *
import time
class Tets():
    def __init__(self):
        self.odemetry_x=0.
        self.odemetry_y=0.
        self.odemetry_theta=0.0
        self.car_length=1
        self.odemetry_vel=0.000001
    def andiff(self,th1,th2):
        d=th1-th2
        #d = mod(d+pi, 2*pi) - pi;
        print "----d------",d
        return (d+pi)%(2.0*pi) - pi
    def Bycycle_Model(self,Vel,Gamma_rad,dt):
            # Vel=0.000001
            # theta=0
            # x=0
            # y=0
            # dt=0.00000001
            
            # if Vel!=0:
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
    def set_pdemetry_vel(self,vel):
        self.odemetry_vel=vel
    def set_pdemetry_x(self,x):
        self.odemetry_x=x
    def set_pdemetry_y(self,y):
        self.odemetry_y=y
    def set_pdemetry_theta(self,theta):
        self.odemetry_theta=theta  
    def Control_mobile_to_one_target(self,x,y,theta,Kv,Kh,dt):
        """
        World coordinate:[x,y,theta]
        """

        Vstar=Kv*sqrt((x-self.odemetry_x)**2+(y-self.odemetry_y)**2)
        # print "self.odemetry_x",self.odemetry_x
        print "x-self.odemetry_x,y-self.odemetry_y",x-self.odemetry_x,y-self.odemetry_y
        thetastar=atan2((y-self.odemetry_y),(x-self.odemetry_x))
        print "thetastar,theta",thetastar,theta
        gammastar=Kh*self.andiff(thetastar,theta)#Kh>0
        
        self.Bycycle_Model(Vstar,gammastar,dt)
        print "------gammastar-----",gammastar,Vstar,thetastar
        # self.pub_vstar.publish(Vstar)
        # self.pub_x.publish(self.odemetry_x)
        # self.pub_y.publish(self.odemetry_y)
        # self.pub_theta.publish(self.odemetry_theta)
def main():
    T=Tets()
    # print T.andiff(-2.876,3.1586)
    x0=[8,5,pi/4]
    count=1
    T.set_pdemetry_x(x0[0])
    T.set_pdemetry_y(x0[1])
    T.set_pdemetry_theta(x0[2])
    while count>0:
        T.Control_mobile_to_one_target(5,5,0.0,0.5,0.4,0.1)
        # count-=1
        time.sleep(0.7)
if __name__=="__main__":
    main()
