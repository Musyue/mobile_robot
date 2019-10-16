#! /usr/bin/env python
# coding=utf-8
# import matplotlib.pyplot as plt
# import numpy as np
# fig = plt.figure(1)
# ax0 = fig.add_subplot(1, 1, 1)
# data = np.arange(10)
# ax0.plot(data)
# #创建一个figure
# fig = plt.figure(2)
# #创建的图像是2*2的，目前选中的是4个subplot中的第一个（编号从1开始）
# ax1 = fig.add_subplot(2, 1, 1)
# # ax3 = fig.add_subplot(2, 2, 3)
# ax2 = fig.add_subplot(2, 1, 2)
# #此时，发出一个绘图指令，matplotlib会在最后一个用过的subplot(若无，则创建一个)上进行绘制
# #'k--'是一个线性选项，代表黑色虚线
# plt.plot(np.random.randn(50).cumsum(), 'k--')
# #柱状图
# ax1.hist(np.random.randn(100), bins=20, color='k', alpha=0.3)
# #散点图
# # ax3.plot(np.arange(30), np.arange(30) + 3 * np.random.randn(30))
# #可以使用plt.close()方法关闭图在控制台出现
# #plt.close('all')
# plt.show()
import rosbag
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import matplotlib.pyplot as plt

bag = rosbag.Bag('/ndata/data/2019-10-10-19-48-20.bag')
temp=[]
result={}
yow=[]
for topic, msg, t in bag.read_messages(topics=['/mobile_base_path']):
    temp.append(msg.poses)
    # print 
# for i in temp:
#     print len(temp[0])
for i in range(len(temp[0])):
    r,p,y=euler_from_quaternion([temp[0][i].pose.orientation.x,temp[0][i].pose.orientation.y,temp[0][i].pose.orientation.z,temp[0][i].pose.orientation.w])
    result.update({i:[(temp[0][i].pose.position.x,temp[0][i].pose.position.y,temp[0][i].pose.position.z),(r,p,y)]})
    yow.append(y)
    # print r,p,y
    # print temp[0][i].pose
# print yow
x=np.array(range(len(yow)))
print x
f1 = np.polyfit(x, yow, 60)
p1 = np.poly1d(f1)
yvals = p1(x)
# print('f1 is :\n',f1)
plot1 = plt.plot(x, yow, 's',label='original values')
plot2 = plt.plot(x, yvals, 'r',label='polyfit values')
plt.show()
bag.close()