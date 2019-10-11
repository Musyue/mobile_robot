#! /usr/bin/env python
# coding=utf-8
import matplotlib.pyplot as plt
import numpy as np
fig = plt.figure(1)
ax0 = fig.add_subplot(1, 1, 1)
data = np.arange(10)
ax0.plot(data)
#创建一个figure
fig = plt.figure(2)
#创建的图像是2*2的，目前选中的是4个subplot中的第一个（编号从1开始）
ax1 = fig.add_subplot(2, 1, 1)
# ax3 = fig.add_subplot(2, 2, 3)
ax2 = fig.add_subplot(2, 1, 2)
#此时，发出一个绘图指令，matplotlib会在最后一个用过的subplot(若无，则创建一个)上进行绘制
#'k--'是一个线性选项，代表黑色虚线
plt.plot(np.random.randn(50).cumsum(), 'k--')
#柱状图
ax1.hist(np.random.randn(100), bins=20, color='k', alpha=0.3)
#散点图
# ax3.plot(np.arange(30), np.arange(30) + 3 * np.random.randn(30))
#可以使用plt.close()方法关闭图在控制台出现
#plt.close('all')
plt.show()