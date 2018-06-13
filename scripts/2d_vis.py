#!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import tf
import os
import sys
from math import cos, sin

goods = [
	# '2017-09-11-19-59-24.bag',

	'2017-09-11-18-54-21.bag',
	'2017-09-11-19-16-41.bag',

	'2017-09-11-19-00-42.bag',
	'2017-09-11-19-09-02.bag',
	'2017-09-11-19-26-50.bag',
	'2017-09-11-19-29-20.bag',
	'2017-09-11-19-32-21.bag',
	'2017-09-11-19-41-14.bag',
	'2017-09-11-19-50-51.bag',
	'2017-09-11-19-57-13.bag',
	'2017-09-11-20-04-41.bag',
	'2017-09-11-20-07-36.bag'
]

failed = [
	'2017-09-11-18-57-03.bag',
	'2017-09-11-19-19-50.bag',
#	'2017-09-11-19-23-06.bag',
	#'2017-09-11-19-34-46.bag',
#	'2017-09-11-19-39-12.bag',
]

"""
goods = [
	'2017-09-13-18-01-49.bag',
	'2017-09-13-18-22-29.bag',
	'2017-09-13-18-03-16.bag',
	'2017-09-13-18-26-40.bag',
	'2017-09-13-18-04-56.bag',
	'2017-09-13-18-28-04.bag',
	'2017-09-13-18-06-48.bag',
	'2017-09-13-18-29-27.bag',
	'2017-09-13-18-08-20.bag',
	'2017-09-13-18-31-35.bag',
	'2017-09-13-18-09-42.bag',
	'2017-09-13-18-32-59.bag',
	'2017-09-13-18-13-26.bag',
	'2017-09-13-18-37-46.bag',
	'2017-09-13-18-16-53.bag',
	'2017-09-13-18-39-08.bag',
	'2017-09-13-18-18-11.bag',
	'2017-09-13-18-40-32.bag',
	'2017-09-13-18-19-38.bag',
	'2017-09-13-18-41-53.bag',
	'2017-09-13-18-21-04.bag',
	'2017-09-13-18-43-21.bag',
	'2017-09-13-18-53-44.bag'
]
"""

repeat_counter = 0
fig = plt.figure(2)
ax = fig.add_subplot(111)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')

def plotBag(bag_file, teach, failed):
	global repeat_counter, fig, ax
	quiver_counter = 0

	x_ = []
	y_ = []
	yaw_ = []

	avg_x = 0.
	avg_y = 0.
	avg_yaw = 0.

	temp_x = []
	temp_y = []
	temp_yaw = []

	bag = rosbag.Bag(bag_file)
	for topic, msg, t in bag.read_messages(topics=['/vicon/bebop_vtr/bebop_vtr']):
		quaternion = (
			msg.transform.rotation.x,
			msg.transform.rotation.y,
			msg.transform.rotation.z,
			msg.transform.rotation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		temp_x.append(msg.transform.translation.x)
		temp_y.append(msg.transform.translation.y)
		yaw = euler[2]
		yaw = yaw - .4 # vicon offset
		if (len(temp_yaw) < 3 or abs(sum(temp_yaw) / float(len(temp_yaw)) - yaw) < 0.5):
			temp_yaw.append(yaw)
		if (quiver_counter < 1 and len(temp_x) > 10) or (quiver_counter >= 1 and len(temp_x) > 150):
			avg_x = sum(temp_x) / float(len(temp_x))
			avg_y = sum(temp_y) / float(len(temp_y))
			avg_yaw = sum(temp_yaw) / float(len(temp_yaw))


			if (quiver_counter < 1):
				r = 0.4
				u = r * cos(avg_yaw)
				v = r * sin(avg_yaw)
				# ax.quiver(avg_x, avg_y, u, v, angles='xy', scale_units='xy', scale=1.2)
				ax.quiver(avg_y, avg_x, v, u, angles='xy', scale_units='xy', scale=1.2)
				quiver_counter = quiver_counter+1
			else:
				x_.append(avg_x)
				y_.append(avg_y)
				yaw_.append(avg_yaw)


#			x_.append(avg_x)
#			y_.append(avg_y)
#			yaw_.append(avg_yaw)

			temp_x = []
			temp_y = []
			temp_yaw = []

	bag.close()
	if teach == True:
		# plt.plot(x_, y_, label="Teaching Trail", linewidth=7.0)
		plt.plot(y_, x_, label="Teaching Trail", linewidth=7.0)
	else:
	#	repeat_counter = repeat_counter + 1
		if failed == True:
			plt.plot(y_, x_, linewidth=3.0)
		else:
			plt.plot(y_, x_, linewidth=1.0)

if __name__ == '__main__':
	repeat_bag_dir = '/local_home/faraz/data/experiments/repeating/11SEP7PM/'
#	repeat_bag_dir = '/local_home/faraz/data/experiments/repeating/13SEP13PM/diff/'
	teach_bag_file = '/local_home/faraz/data/experiments/teaching/2017-09-11-18-45-37.bag'

	plotBag(teach_bag_file, teach=True, failed=False)
	for index in range(len(goods)):
		plotBag(os.path.join(repeat_bag_dir, goods[index]), teach=False, failed=False)

	for index in range(len(failed)):
		plotBag(os.path.join(repeat_bag_dir, failed[index]), teach=False, failed=True)



	#ax.quiver((0,0,1), (1, 1, 2),angles='xy', scale_units='xy', scale = 1)
	#plt.xticks(range(-3,4))
	#plt.yticks(range(-3,4))
	plt.legend()
	plt.gca().set_aspect('equal', adjustable='box')
	plt.gca().invert_yaxis()
	plt.xlabel('X (m)', fontsize=18)
	plt.ylabel('Y (m)', fontsize=16)
	plt.grid()
	plt.draw()
	#plt.scatter(x_, y_)
	# plt.show()
	plt.savefig("2d_fig.png")
