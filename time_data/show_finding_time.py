# coding: utf-8

import matplotlib.pyplot as plt
import numpy as np

ft = np.loadtxt("finding_time.txt")
med = np.median(ft)
ave = np.average(ft)
sum = np.sum(ft)
print("med:{} \nave:{} \nsum:{}".format(med, ave, sum))
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
boxdic = {
    "facecolor": "white",
    "edgecolor": "white",
    "linewidth": 0
}
ax.axvline(med, ls="--", color="green")
ax.text(med + 5, 11, "Median: {:.2f}s".format(med),
        size=10, color="green", bbox=boxdic)
ax.axvline(ave, ls="--", color="red")
ax.text(ave + 5, 12, "Average: {:.2f}s".format(ave),
        size=10, color="red", bbox=boxdic)
ax.hist(ft, bins=50)
ax.set_xlabel('Time to find s')
fig.show()
