#g!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import tf
from math import cos, sin
from scipy.stats import norm


#data=[48,55,52,50,51,52,55,52,51,52,52,56,48,44,55,53,54,51]
data=[44,55,52,49,50,51,55,52,50,52,51,55,48,44,55,53,54,50]
mu, std = norm.fit(data)
# Plot the histogram.
plt.hist(data, bins=25, normed=True, alpha=0.6, color='g')

# Plot the PDF.
xmin, xmax = plt.xlim()
x = np.linspace(xmin, xmax, 100)
p = norm.pdf(x, mu, std)
plt.plot(x, p, 'k', linewidth=2)
title = "Fit results: mu = %.2f,  std = %.2f" % (mu, std)
plt.title(title)

plt.show()
