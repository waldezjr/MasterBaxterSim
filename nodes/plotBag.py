#!/usr/bin/env python

import rosbag
from master_baxter_sim.msg import BaxterArmLog
import matplotlib.pyplot as plt
import os
import numpy as np
from math import (pi,sin,cos,exp,tan,sqrt)

def circular_traj(t,x_c,tSim, mirror=1):
    # 'mirror' will mirror the trajectory with respect to the x_base axis
    omega = 2*pi/tSim
    r = 0.05 #radius 
    if t<tSim:
        x_ref = np.matrix([x_c[0]+r*cos(omega*t),x_c[1]+mirror*r*sin(omega*t),x_c[2]])
        x_ref_dot = np.matrix([-r*omega*sin(omega*t),mirror*r*omega*cos(omega*t),0.0])
        x_ref_dot_dot = np.matrix([-r*omega*omega*cos(omega*t),-mirror*r*omega*omega*sin(omega*t),0.0])
    else:
        x_ref = np.matrix([x_c[0]+r,x_c[1],x_c[2]])
        x_ref_dot = np.matrix([0.0,0.0,0.0])
        x_ref_dot_dot = np.matrix([0.0,0.0,0.0])
    return x_ref, x_ref_dot,x_ref_dot_dot

def human_traj(t,x_c,tSim):
    t_delta = float(tSim)/8 #Mind the float division
    r = 0.05 #radius

    x_d = np.matrix([0.0,0.0,0.0])
    p1, aux1, aux2 = circular_traj(t_delta,x_c,tSim)
    p2 = np.matrix([x_c[0], x_c[1] + 3*r/2, x_c[2]])
    p3, aux1, aux2 = circular_traj(3*t_delta,x_c,tSim)
    p4, aux1, aux2 = circular_traj(5*t_delta,x_c,tSim)

    if t < t_delta:
        x_d, x_d_dot, x_d_dot_dot = circular_traj(t,x_c,tSim)

    elif t >= t_delta and t < 2*t_delta:
        x_d = (t-t_delta) * (p2-p1)/t_delta + p1;

    elif t >= 2*t_delta and t < 3*t_delta:
        x_d = (t-2*t_delta) * (p3-p2)/t_delta + p2;

    elif t >= 3*t_delta and t < 5*t_delta:
        x_d = (t-3*t_delta) * (p4-p3)/(2*t_delta) + p3;

    elif t >= 5*t_delta and t < tSim:
        x_d, x_d_dot, x_d_dot_dot = circular_traj(t,x_c,tSim)

    else:
        x_d, x_d_dot, x_d_dot_dot = circular_traj(0,x_c,tSim)

    return x_d

x_c = [0.7,0.25,0.15]


# folder_str = os.getcwd() + '/src/MasterBaxterSim/nodes/Bags/'
folder_str = os.getcwd() + '/src/master_baxter_sim/nodes/Bags/'

t0 = float(-1)
time_vec0 = []
time_vec05 = []
time_vec1 = []
time_vec_a = []
x_x0=[]
x_y0=[]
x_z0=[]

e_r0=[]
e_h0=[]


x_x05=[]
x_y05=[]
x_z05=[]

e_r05=[]
e_h05=[]


x_x1=[]
x_y1=[]
x_z1=[]

e_r1=[]
e_h1=[]

x_x_a=[]
x_y_a=[]
x_z_a=[]

e_r_a=[]
e_h_a=[]


alpha = []
Fh_x = []
Fh_y = []

x_r_x =[]
x_r_y =[]
x_h_x =[]
x_h_y =[]



bag = rosbag.Bag(folder_str + 'new_alpha0.bag')
for topic, msg, t in bag.read_messages(topics=['left_arm_log']):
    #get initial t
    if t0==-1:
        t0 = t.to_sec()
    #get current t
    t_cur = t.to_sec()-t0
    time_vec0.append(t_cur)

    x_x0.append(msg.EEF_pos.x)
    x_y0.append(msg.EEF_pos.y)
    x_z0.append(msg.EEF_pos.z)

    e_r0.append(msg.robot_error_pos)
    e_h0.append(msg.human_error_pos)

    aux1, aux2,aux3 = circular_traj(t_cur,x_c,30)
    x_r_x.append(aux1[0,0])
    x_r_y.append(aux1[0,1])

    aux1 = human_traj(t_cur,x_c,30)
    x_h_x.append(aux1[0,0])
    x_h_y.append(aux1[0,1])

bag.close()
t0=-1

bag = rosbag.Bag(folder_str + 'new_alpha05.bag')
for topic, msg, t in bag.read_messages(topics=['left_arm_log']):
    #get initial t
    if t0==-1:
        t0 = t.to_sec()
    #get current t
    t_cur = t.to_sec()-t0    

    time_vec05.append(t_cur)
    x_x05.append(msg.EEF_pos.x)
    x_y05.append(msg.EEF_pos.y)
    x_z05.append(msg.EEF_pos.z)

    e_r05.append(msg.robot_error_pos)
    e_h05.append(msg.human_error_pos)
bag.close()
t0=-1

bag = rosbag.Bag(folder_str + 'new_alpha1.bag')
for topic, msg, t in bag.read_messages(topics=['left_arm_log']):
    #get initial t
    if t0==-1:
        t0 = t.to_sec()
    #get current t
    t_cur = t.to_sec()-t0    

    time_vec1.append(t_cur)
    x_x1.append(msg.EEF_pos.x)
    x_y1.append(msg.EEF_pos.y)
    x_z1.append(msg.EEF_pos.z)
    e_r1.append(msg.robot_error_pos)
    e_h1.append(msg.human_error_pos)
bag.close()
t0=-1

bag = rosbag.Bag(folder_str + 'new_alpha_adapt.bag')
for topic, msg, t in bag.read_messages(topics=['left_arm_log']):
    #get initial t
    if t0==-1:
        t0 = t.to_sec()
    #get current t
    t_cur = t.to_sec()-t0    

    time_vec_a.append(t_cur)
    x_x_a.append(msg.EEF_pos.x)
    x_y_a.append(msg.EEF_pos.y)
    x_z_a.append(msg.EEF_pos.z)
    alpha.append(msg.alpha)

    e_r_a.append(msg.robot_error_pos)
    e_h_a.append(msg.human_error_pos)

    Fh_x.append(msg.f_ext.x)
    Fh_y.append(msg.f_ext.y)
bag.close()


line_wdt = 3
plt.rc('font', family='serif')
plt.rcParams.update({'font.size': 30})
# plot trajectories
plt.figure(1,figsize=(14,10))

# plt.plot(x_x0,x_y0, label = r'$\alpha$= 0', lw = line_wdt)#, x_x05,x_y05,x_x1,x_y1,x_x_a,x_y_a, aa=True)
# plt.plot(x_x05,x_y05, label = r'$\alpha$= 0.5', lw = line_wdt)
# plt.plot(x_x1,x_y1, label = r'$\alpha$= 1',  lw =line_wdt)
plt.plot(x_h_x,x_h_y, label = r'$x_h(t)$', lw =line_wdt)
plt.plot(x_r_x,x_r_y, label = r'$x_r(t)$', lw =line_wdt)
# plt.plot(x_x_a,x_y_a, label = r'$\alpha$' + ' adaptive', lw =line_wdt)
plt.plot(x_x_a,x_y_a, label = r'$x_e(t)$', lw =line_wdt)
# print x_h_x
plt.axis([0.63, 0.76, 0.18, 0.34])
plt.title('Trajectories X-Y')
plt.xlabel(r'$x_b$ (m)')
plt.ylabel(r'$y_b$ (m)')
plt.legend(loc =4)


# plot alpha
plt.figure(2,figsize=(14,10))
plt.plot(time_vec_a,alpha,lw = line_wdt, label = r'$\alpha(t)$')
# plt.title(r'$\alpha(t)$ ')
plt.xlabel('time (s)')
plt.ylabel(r'$\alpha$')
plt.legend()
plt.axis([0, 45, -.1, 1.1])

# plot errors alpha 0
plt.figure(3)
plt.plot(time_vec0,e_h0,lw = line_wdt, label = r'$\parallel e_h(t) \parallel $')
plt.plot(time_vec0,e_r0,lw = line_wdt, label = r'$\parallel e_r(t) \parallel $')
plt.xlabel('time (s)')
plt.ylabel('norm (m)')
plt.title(r'Error Norms ($\alpha=0$) ')
plt.legend()

# plot errors alpha 0.5
plt.figure(4)
plt.plot(time_vec05,e_h05,lw = line_wdt, label = r'$\parallel e_h(t) \parallel $')
plt.plot(time_vec05,e_r05,lw = line_wdt, label = r'$\parallel e_r(t) \parallel $')
plt.xlabel('time (s)')
plt.ylabel('norm (m)')
plt.title(r'Error Norms ($\alpha=0.5$) ')
plt.legend()

# plot errors alpha 1.0
plt.figure(5)
plt.plot(time_vec1,e_h1,lw = line_wdt, label = r'$\parallel e_h(t) \parallel $')
plt.plot(time_vec1,e_r1,lw = line_wdt, label = r'$\parallel e_r(t) \parallel $')
plt.xlabel('time (s)')
plt.ylabel('norm (m)')
plt.title(r'Error Norms ($\alpha=1$) ')
plt.legend()

# plot errors alpha adaptive
plt.figure(6,figsize=(14,10))
plt.plot(time_vec_a,e_h_a,lw = line_wdt, label = r'$\parallel e_h(t)\parallel $')
plt.plot(time_vec_a,e_r_a,lw = line_wdt, label = r'$\parallel e_r(t)\parallel $')
plt.xlabel('time (s)')
plt.ylabel('norm (m)')
plt.title(r'Error Norms ')
plt.legend()

# external force x

plt.figure(7)
plt.plot(time_vec_a,Fh_x,lw = line_wdt, label = r'$F_{hx}$')
plt.plot(time_vec_a,Fh_y,lw = line_wdt, label = r'$F_{hy}$')
plt.title('Simulated Human Force')
plt.xlabel('time (s)')
plt.ylabel('Force (N)')
plt.legend()



plt.show()