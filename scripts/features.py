#!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import tf
import os
import sys
from math import cos, sin

files1 = [
#	'2017-09-12-10-35-42.bag',
	'2017-09-12-10-55-51.bag',  
#	'2017-09-12-10-57-51.bag',  
#	'2017-09-12-10-59-56.bag',  
#	'2017-09-12-11-24-08.bag',  

#	'2017-09-12-11-20-25.bag',  
#	'2017-09-12-11-32-14.bag',  
#	'2017-09-12-11-50-18.bag'
#	'2017-09-12-11-05-44.bag', 
#	'2017-09-12-11-30-23.bag',  
#	'2017-09-12-11-54-06.bag',
#	'2017-09-12-11-11-23.bag',  
#	'2017-09-12-11-22-25.bag',  
#	'2017-09-12-11-33-57.bag',  
#	'2017-09-12-11-01-46.bag',  
#	'2017-09-12-11-13-07.bag',  
]

repeat_bag_dir1 = '/local_home/faraz/data/experiments/repeating/12SEP10AM-20SIMPLE/'

files2 = [
#	'2017-09-13-13-12-51.bag',
	'2017-09-13-13-17-21.bag',
#	'2017-09-13-13-14-17.bag'
]

repeat_bag_dir2 = '/local_home/faraz/data/experiments/repeating/13SEP13PM/diff_height/'

files3 = [
#	'2017-09-13-12-47-21.bag',
	'2017-09-13-13-01-13.bag',
#	'2017-09-13-12-48-45.bag'
]

repeat_bag_dir3 = '/local_home/faraz/data/experiments/repeating/13SEP13PM/diff_light/'

files4 = [
#	'2017-09-13-18-01-49.bag',
	'2017-09-13-18-26-40.bag',
#	'2017-09-13-18-03-16.bag'
]

repeat_bag_dir4 = '/local_home/faraz/data/experiments/repeating/13SEP13PM/diff/'

repeat_counter = 0
fig = plt.figure(2)
ax = fig.add_subplot(111)
ax.set_xlabel('Time (s)', fontsize=14)
ax.set_ylabel('Number of Features', fontsize=14)

def plotBag(bag_file, teach, label):
	global repeat_counter, fig, ax

	feature_counter = []
	timer = []
	temp_feature_counter = []
	bag = rosbag.Bag(bag_file)
	base_time = 0
	for topic, msg, t in bag.read_messages(topics=['/vision/yolo2/detections', '/joy']):
		if topic == '/joy':
			if msg.buttons[0] == 1:
				break
		else:
			if (len(temp_feature_counter) > 60):
				feature_counter.append(sum(temp_feature_counter) / float(len(temp_feature_counter)))
				temp_feature_counter = []

				if len(timer) == 0:
					timer.append(0)
					base_time = msg.header.stamp.secs
				else:
					timer.append(msg.header.stamp.secs - base_time)
			else:
				temp_feature_counter.append(len(msg.detections))

	bag.close()
	if teach == True:
		plt.plot(timer, feature_counter, label="Teaching", linewidth=3.0)
	else:
		repeat_counter = repeat_counter + 1
		plt.plot(timer, feature_counter, label=label, linewidth=1.0)

if __name__ == '__main__':
	# repeat_bag_dir = '/local_home/faraz/data/experiments/repeating/11SEP7PM/'
#	teach_bag_file = '/local_home/faraz/data/experiments/teaching/2017-09-11-18-45-37.bag'

	teach_bag_file = '/local_home/faraz/data/experiments/repeating/12SEP10AM-20SIMPLE/2017-09-12-11-18-01.bag'

	# plotBag(teach_bag_file, teach=True, label="")
	for index in range(len(files1)):
		plotBag(os.path.join(repeat_bag_dir1, files1[index]), teach=False, label="No Changes ")
	for index in range(len(files2)):
		plotBag(os.path.join(repeat_bag_dir2, files2[index]), teach=False, label="Different Height ")
	for index in range(len(files3)):
		plotBag(os.path.join(repeat_bag_dir3, files3[index]), teach=False, label="Different Lighting ")
	for index in range(len(files4)):
		plotBag(os.path.join(repeat_bag_dir4, files4[index]), teach=False, label="Different Objects ")

	#ax.quiver((0,0,1), (1, 1, 2),angles='xy', scale_units='xy', scale = 1)
	ax.set_ylim(ymin=0)
	ax.set_xlim(xmin=0, xmax=55)
	plt.legend()
	plt.grid()
	plt.draw()
	#plt.scatter(x_, y_)
	# plt.show()
	plt.savefig('foo.png')
