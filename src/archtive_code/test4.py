from datetime import datetime
from math import *
import time
v=0.1
L=0.5
theta=pi/3
count=0

while count<10:
    xstar=v*cos(theta)
    ystar=v*sin(theta)
    thetastar=v*tan(theta)/L
    print "xstar,ystar,thetastar",xstar,ystar,thetastar
    # a=datetime.now()
    a=time.time()
    print "opreating bicycle------"
    time.sleep(0.5)
    # b=datetime.now()
    b=time.time()
    dt=(b-a)
    print b-a
    theta=thetastar*dt
    print "dt,xstar*dt,ystar*dt,theta",dt,xstar*dt,ystar*dt,theta
    count+=1
    print count
    