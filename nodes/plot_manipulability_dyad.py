import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import csv
import os

folder_manip = '../../PostProcessed_Data/Manipulability/Dyad/'

color = {'0L' : 'g', '1L' : 'b', '2L' : 'r'}

nb_dyads = 7
nrows = 2
ncols = 4

figs = dict()
for s in ['S1', 'S2']:
	fig, ax = plt.subplots(nrows=nrows, ncols=ncols)
	count = 0
	for r in ax:
		for c in r:
			if count < nb_dyads:
				c.set_xlabel('Time (s)', fontsize=20, fontweight='bold')
				c.set_ylabel('Manipulability', fontsize=20, fontweight='bold')
				c.set_xlim([0,11])
				c.xaxis.set_major_locator(MaxNLocator(4))
				c.yaxis.set_major_locator(MaxNLocator(4))
				count += 1
			else:
				fig.delaxes(c)
	fig.suptitle('Manipulability ' + s , fontsize=28, fontweight='bold')
	fig.subplots_adjust(left=0.06, bottom=0.06, right=0.99, top=0.89, wspace=0.3, hspace=0.3)
	figs[s] = [fig, ax]

for f in sorted(os.listdir(folder_manip)):
	dyad = f.split('_')[1]
	cond = f.split('_')[3]
 	subject = f.split('_')[7].split('.')[0]
 	# print dyad

	with open(folder_manip + '/' + f, 'r') as fin:
		data = np.genfromtxt(fin, delimiter=',')	

	# for i in range(nb_dyads):
		# print i
	figs[subject][1][(int(dyad)-2)/ncols, (int(dyad)-2)%ncols].plot(data[:,0], data[:,1], color[cond])
	figs[subject][1][(int(dyad)-2)/ncols, (int(dyad)-2)%ncols].set_title('Dyad '+dyad, fontsize=20, fontweight='bold')

plt.show()
