#g!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import tf
from math import cos, sin
fig = plt.figure(2)
ax = fig.add_subplot(111)

filelist1 =[
'2017-09-12-10-35-42.bag', 
# '2017-09-12-10-53-01.bag', 
# '2017-09-12-11-03-45.bag', 
#'2017-09-12-11-15-29.bag', 
# '2017-09-12-11-26-00.bag',  
#'2017-09-12-11-52-06.bag',
'2017-09-12-10-37-28.bag',  
'2017-09-12-10-55-51.bag',  
'2017-09-12-11-05-44.bag', 
 '2017-09-12-11-18-01.bag',  
'2017-09-12-11-30-23.bag',  
'2017-09-12-11-54-06.bag',
#'2017-09-12-10-40-12.bag',  
'2017-09-12-10-57-51.bag',  
#'2017-09-12-11-07-39.bag',  
'2017-09-12-11-20-25.bag',  
'2017-09-12-11-32-14.bag',  
#'2017-09-12-11-56-04.bag', 
#'2017-09-12-10-42-54.bag',  
'2017-09-12-10-59-56.bag',  
'2017-09-12-11-11-23.bag',  
'2017-09-12-11-22-25.bag',  
'2017-09-12-11-33-57.bag',  
#'2017-09-12-11-57-58.bag',
#'2017-09-12-10-49-15.bag',  
'2017-09-12-11-01-46.bag',  
'2017-09-12-11-13-07.bag',  
'2017-09-12-11-24-08.bag',  
'2017-09-12-11-50-18.bag']


inputdir1='/local_home/faraz/data/experiments/repeating/12SEP10AM-20SIMPLE/'

filelist2 =[
	'2017-09-13-18-01-49.bag',
	'2017-09-13-18-22-29.bag',
	'2017-09-13-18-03-16.bag',
	'2017-09-13-18-26-40.bag',
	'2017-09-13-18-04-56.bag',
	'2017-09-13-18-28-04.bag',
	'2017-09-13-18-06-48.bag',
#	'2017-09-13-18-29-27.bag',
	'2017-09-13-18-08-20.bag',
#	'2017-09-13-18-31-35.bag',
	'2017-09-13-18-09-42.bag',
	'2017-09-13-18-32-59.bag',
#	'2017-09-13-18-13-26.bag',
	'2017-09-13-18-37-46.bag',
	'2017-09-13-18-16-53.bag',
	'2017-09-13-18-39-08.bag',
	'2017-09-13-18-18-11.bag',
	'2017-09-13-18-40-32.bag',
	'2017-09-13-18-19-38.bag',
	'2017-09-13-18-41-53.bag',
	'2017-09-13-18-21-04.bag',
	'2017-09-13-18-43-21.bag',
#	'2017-09-13-18-53-44.bag'
]

inputdir2='/local_home/faraz/data/experiments/repeating/13SEP13PM/diff/'

filelist3 = [
	'2017-09-13-12-47-21.bag',
	'2017-09-13-13-01-13.bag',
	'2017-09-13-12-48-45.bag',
	'2017-09-13-13-02-26.bag',
	'2017-09-13-12-50-54.bag',
	'2017-09-13-13-04-31.bag',
	'2017-09-13-12-53-58.bag',
	'2017-09-13-13-05-42.bag',
	'2017-09-13-12-55-19.bag',
	'2017-09-13-13-07-38.bag'
]

inputdir3='/local_home/faraz/data/experiments/repeating/13SEP13PM/diff_light/'

#bag = rosbag.Bag(inputdir + filelist[1])
#bag = rosbag.Bag('/local_home/faraz/data/experiments/teaching/2017-09-11-18-45-37.bag')
'''
temp_x = []
temp_y = []
temp_yaw = []
quiver_counter = 0

'''
time_s=[]
time_ms=[]
time_diff1=[]
time_diff2=[]
time_diff3=[]
i=0
print(len(filelist1) + len(filelist2) + len(filelist3))
print "total naumber of test\n"

for i in range(len(filelist1)):
	time_s = []
	time_ms= []
	bag = rosbag.Bag(inputdir1 + filelist1[i])
	for topic, msg, t in bag.read_messages(topics=['/joy']):
		if len(time_s) < 1:
			time_s.append(msg.header.stamp.secs)
			time_ms.append(msg.header.stamp.nsecs)
		elif msg.header.stamp.secs > time_s[-1]+10:
			time_s.append(msg.header.stamp.secs)
			time_ms.append(msg.header.stamp.nsecs)
			print float(time_s[-1]-time_s[-2])+(float((time_ms[-1])-float(time_ms[-2]))/10**9)
			time_diff1.append(float(time_s[-1]-time_s[-2])+(float((time_ms[-1])-float(time_ms[-2]))/10**9))
			#print time_s[-1]-time_s[-2]	
	print "test number is" 
	print i
	print "\n"
	print "\n"

	bag.close()


for i in range(len(filelist2)):
	time_s = []
	time_ms= []
	bag = rosbag.Bag(inputdir2 + filelist2[i])
	for topic, msg, t in bag.read_messages(topics=['/joy']):
		if len(time_s) < 1:
			time_s.append(msg.header.stamp.secs)
			time_ms.append(msg.header.stamp.nsecs)
		elif msg.header.stamp.secs > time_s[-1]+10:
			time_s.append(msg.header.stamp.secs)
			time_ms.append(msg.header.stamp.nsecs)
			print float(time_s[-1]-time_s[-2])+(float((time_ms[-1])-float(time_ms[-2]))/10**9)
			time_diff2.append(float(time_s[-1]-time_s[-2])+(float((time_ms[-1])-float(time_ms[-2]))/10**9))
			#print time_s[-1]-time_s[-2]	
	print "test number is" 
	print i
	print "\n"
	print "\n"

	bag.close()

for i in range(len(filelist3)):
	time_s = []
	time_ms= []
	bag = rosbag.Bag(inputdir3 + filelist3[i])
	for topic, msg, t in bag.read_messages(topics=['/joy']):
		if len(time_s) < 1:
			time_s.append(msg.header.stamp.secs)
			time_ms.append(msg.header.stamp.nsecs)
		elif msg.header.stamp.secs > time_s[-1]+10:
			time_s.append(msg.header.stamp.secs)
			time_ms.append(msg.header.stamp.nsecs)
			print float(time_s[-1]-time_s[-2])+(float((time_ms[-1])-float(time_ms[-2]))/10**9)
			time_diff3.append(float(time_s[-1]-time_s[-2])+(float((time_ms[-1])-float(time_ms[-2]))/10**9))
			#print time_s[-1]-time_s[-2]	
	print "test number is" 
	print i
	print "\n"
	print "\n"

	bag.close()



from scipy.stats import norm


#data=[48,55,52,50,51,52,55,52,51,52,52,56,48,44,55,53,54,51]
#data=[44,55,52,49,50,51,55,52,50,52,51,55,48,44,55,53,54,50]

# Plot the histogram.
bins = np.linspace(40, 65, 25)
plt.hist(time_diff1, bins=bins, normed=True, alpha=0.6, color='g', label='No changes')
plt.hist(time_diff2, bins=bins, normed=True, alpha=0.6, color='b', label='Some Objects have been Removed/Moved')
plt.hist(time_diff3, bins=bins, normed=True, alpha=0.6, color='r', label='Different Lighting, Same Objects')

# Plot the PDF.
mu1, std1 = norm.fit(time_diff1)
mu2, std2 = norm.fit(time_diff2)
mu3, std3 = norm.fit(time_diff3)
xmin, xmax = plt.xlim()
x = np.linspace(xmin, xmax, 100)
p1 = norm.pdf(x, mu1, std1)
p2 = norm.pdf(x, mu2, std2)
p3 = norm.pdf(x, mu3, std3)
plt.plot(x, p1, 'k', linewidth=2)
plt.plot(x, p2, 'k', linewidth=2)
plt.plot(x, p3, 'k', linewidth=2)
plt.legend()
title = "Fit results: mu_green = %.2f, mu_blue = %.2f, mu_red = %.2f" % (mu1, mu2, mu3)
plt.title(title)
ax.set_xlabel('Travelling Time (s)')
ax.set_ylabel('Frequency')

#plt.show()
plt.savefig("normality.png")
