#!/usr/bin/env python
import matplotlib
import matplotlib.gridspec as gridspec
from pylab import *
import matplotlib.pyplot as plt
import numpy as np
from math import pow, exp
from scipy.stats import norm

# 133 0.5

#Create test data with zero valued diagonal:
data = np.genfromtxt("seq.txt", delimiter=', ')
data = data[125-20:125+20,]
data = data[::-1]
# data = data.reshape((20,3,20,-1)).mean(axis=3).mean(1)
data = (data - data.min()) * (1. / (data.max() - data.min()))
#rows, cols = np.indices((60,30))

#Create new colormap, with white for zero 
#(can also take RGB values, like (255,255,255):

colors = [('white')] + [(cm.jet(i)) for i in xrange(1,256)]
new_map = matplotlib.colors.LinearSegmentedColormap.from_list('new_map', colors, N=256)

plt.subplots_adjust(wspace=.0)
gs = gridspec.GridSpec(1, 2, width_ratios=[5, 1]) 

ax0 = plt.subplot(gs[0])
plt.pcolor(data, cmap=new_map)
#plt.gca().set_aspect('equal', adjustable='box')
ax0.set_aspect('equal', adjustable='box')

ax0.text(25, 38, "-0.50", color='white', bbox={'facecolor':'black', 'alpha':0.75, 'pad':2})
ax0.plot([0, 30], [27, 39], 'r--', lw=1, color='black')

ax0.text(25, 33, "-0.25", color='white', bbox={'facecolor':'black', 'alpha':0.75, 'pad':2})
ax0.plot([0, 30], [27, 32], 'r--', lw=1, color='black')

ax0.text(25, 28, "0.00", color='white', bbox={'facecolor':'black', 'alpha':0.75, 'pad':2})
ax0.plot([0, 30], [27, 27], 'r--', lw=1, color='black')

ax0.text(25, 23, "0.25", color='white', bbox={'facecolor':'black', 'alpha':0.75, 'pad':2})
ax0.plot([0, 30], [27, 20], 'r--', lw=1, color='black')

ax0.text(25, 16, "0.50", color='white', bbox={'facecolor':'black', 'alpha':0.75, 'pad':2})
ax0.plot([0, 30], [27, 12], 'k-', lw=1.5, color='black')

ax0.text(25, 10, "0.75", color='white', bbox={'facecolor':'black', 'alpha':0.75, 'pad':2})
ax0.plot([0, 30], [27, 5], 'r--', lw=1, color='black')

ax0.text(25, 3, "1.00", color='white', bbox={'facecolor':'black', 'alpha':0.75, 'pad':2})
ax0.plot([0, 27], [27, 0], 'r--', lw=1, color='black')

plt.title("Comparisons")
plt.xlabel("Current Observations", fontsize=16)
plt.ylabel("Memory", fontsize=16)

ax1 = plt.subplot(gs[1], sharey = ax0)
plt.setp(ax1.get_yticklabels(), visible=False)
plt.setp(ax1.get_xticklabels(), visible=False)
plt.title("Temporallity")

tempo = np.asarray([[exp(-(pow((i-12.), 2.0)) / (2.0 * 80))] for i in range(40)])
# tempo = np.concatenate((np.asarray([[(1. + norm.pdf(i, 22., 10.)) / 2.] for i in xrange(0, 23)]), np.asarray([[(1. + 2. * norm.pdf(i, 22., 10.)) / 3.] for i in xrange(24, 60)])), axis=1)
tempo = (tempo - tempo.min()) * (1. / (tempo.max() - tempo.min()))

plt.pcolor(tempo, cmap=new_map)

y = np.linspace(0, 40, 40)
plt.plot(tempo, y, 'k-', linewidth=1)

plt.colorbar()

#savefig('map.png')
show()
