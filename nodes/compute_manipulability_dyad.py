import numpy as np
import os
import csv
from master_baxter_sim.HHDyad import HumanArm
from math import pi


""" 
Compute manipulabily values (Yoshikawa,1985), for the human arms in the dyads 
"""

joints_folder = '../../PostProcessed_Data/Joint_Angles/Dyad'
arm_length_folder = '../../PostProcessed_Data/Arm_Length/Dyad'
folder_out = '../../PostProcessed_Data/Manipulability/Dyad'
urdf_filename = '/home/waldezjr/catkin_ws/src/master_baxter_sim/robots/human_right_arm.xacro'

# joints 
# DH order: shoulder_rot; shoulder_flex; shoulder_adduction; elbow_flex; elbow_pronation; wrist_flexion;  wrist_adduction
# joint file order: shoulder_flex; shoulder_rot;shoulder_adduction; elbow_flex; elbow_pronation; wrist_flexion;  wrist_adduction

arm_lengths = {'S1': dict(),'S2': dict(), }
with open(arm_length_folder + '/Arm_Length_Dyad.csv', 'r') as fin:
	arm_data = np.genfromtxt(fin, delimiter='\t', skip_header=4)[:,5:]
	# print arm_data.shape[0]
	j = 2
	for i in xrange(0,arm_data.shape[0]):
		if (i)%2==0:
			arm_lengths['S1'][j] = arm_data[i].tolist()
		else:
			arm_lengths['S2'][j] = arm_data[i].tolist()
			j = j + 1


print 'Lengths for Dyad 8, Subject 1:'
print 'Arm: ' , arm_lengths['S1'][8][0] #l1
print 'Forearm: ' , arm_lengths['S1'][8][1] #l2
print 'Hand: ' , arm_lengths['S1'][8][2] #l3

for dyadName in sorted(os.listdir(joints_folder)):
	# print dyadName
 	if dyadName[0:4] == 'Dyad':
 		folder = joints_folder + '/' + dyadName
 		for f in sorted(os.listdir(folder)):
 			# get info from filename
 			cond = f.split('_')[3]
 			trial = f.split('_')[5]
 			subject = f.split('_')[7].split('.')[0]
 			# print cond + ' ' + trial + ' ' + subject

 			#get arm length(from cm to m), and create robot

 			l1 = arm_lengths[subject][int(dyadName[5])][0] / 100
 			l2 = arm_lengths[subject][int(dyadName[5])][1] / 100
 			l3 = arm_lengths[subject][int(dyadName[5])][2] / 100

 			# print l1

 			arm = HumanArm(l1,l2,l3, urdf_filename,subject)

			fname = dyadName + '_Cond_' + cond +'_Trial_' + trial + '_Manipulability_' + subject + '.csv'   

			with open(folder_out + '/' + fname  , 'w') as fp:
				writer = csv.writer(fp, delimiter=',')

	 			#open file and get joint configurations
	 			with open(joints_folder +'/'+ dyadName +'/'+ f, 'r') as fin:
					data = np.genfromtxt(fin, delimiter=',', skip_header=11)

	 				#iterate through various joint configurations
					for i in xrange(1,data.shape[0]):
						time = float(data[i][0])
						joints_from_file = data[i][7:]

						swap_aux = joints_from_file[0]
						joints_from_file[0] = joints_from_file[1]
						joints_from_file[1] = swap_aux

						joints = np.asarray(joints_from_file) * pi / 180
						# print 'joints', joints
	 					#calculate manipulability for a certain joint configuration
	 					manip = arm.get_manipulability(joints)
						writer.writerow([time,manip])




 					






# 		sensors_id = {'S1' : dict(), 'S2' : dict()}
# 		mvc = {'S1' : dict(), 'S2' : dict()}
# 		# Get MVC
# 		folder = data_folder + '/' + subjectName + '/MVC_calibration'
# 		for f in sorted(os.listdir(folder)):
# 			subject = f.split('_')[3]
# 			with open(folder + '/' + f, 'r') as fin:
# 				data = list(csv.reader(fin, delimiter=','))
# 				muscle_names = data[0]
# 				muscle_data = data[1]
# 				for i in range(len(muscle_names)):
# 					sensors_id[subject][muscle_names[i]] = int(muscle_data[2*i])
# 					mvc[subject][muscle_names[i]] = float(muscle_data[2*i+1])

# 		# Compute ICC for all trials
# 		emg_rms = dict()
# 		folder = data_folder + '/' + subjectName + '/EMG_data'
# 		for f in sorted(os.listdir(folder)):
# 			with open(folder + '/' + f, 'r') as fin:
# 				emg_channels = list(csv.reader(fin, delimiter=','))[13][2:]
# 			with open(folder + '/' + f, 'r') as fin:
# 				data = np.genfromtxt(fin, delimiter=',', skip_header=14)	
# 			time = data[:,1]

# 			# Compute RMS with sliding window
# 			for ch in emg_channels:
# 				ch_ID = int(ch.split('CH')[1])
# 				emg_rms[ch_ID] = np.zeros(len(data)-rms_sliding_window_width)
# 				for i in range(len(emg_rms [ch_ID])):
# 					emg_rms[ch_ID][i] = np.linalg.norm(data[i:i+rms_sliding_window_width, emg_channels.index(ch)+2]*10**-6)/np.sqrt(data[i:i+rms_sliding_window_width, emg_channels.index(ch)+2].size)

# 				# Filer
# 				# emg_rms[ch_ID] = sg.filtfilt(b,a,emg_rms[ch_ID])
# 				emg_rms[ch_ID] = sg.lfilter(b,a,emg_rms[ch_ID])

# 			# Compute ICC for both subjects
# 			icc = np.zeros((len(data)-rms_sliding_window_width, 1+2*len(muscle_pairs.keys())))
# 			icc_names = ['Time']
# 			index = 0
# 			icc[:,0] = time[rms_sliding_window_width:]
# 			for s in ['S1', 'S2']:
# 				for segment, muscles in muscle_pairs.items():
# 					icc_names.append(segment + '_' + s)
# 					icc[:, index+1] = np.minimum(emg_rms[sensors_id[s][muscles[0]]] / mvc[s][muscles[0]], emg_rms[sensors_id[s][muscles[1]]] / mvc[s][muscles[1]])	
# 					index += 1

# 			# Write	in file
# 			with open(folder_out + '/' + f.split('EMG')[0] + 'ICC.csv', 'w') as fout:
# 				writer = csv.writer(fout, delimiter=',')
# 				writer.writerow(icc_names)
# 				writer.writerows(icc)
