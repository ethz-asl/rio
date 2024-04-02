#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
plt.rcParams['text.usetex'] = True
plt.rc('text.latex', preamble=r'\usepackage{amsmath}')
import rosbag
from rospy import Time
from nav_msgs.msg import Odometry

# Open rosbag
q_bag = rosbag.Bag('/home/brik/Desktop/paper_analysis/baro_residuals_quadratic.bag')
f_bag = rosbag.Bag('/home/brik/Desktop/paper_analysis/baro_residuals_fair.bag')
no_baro_bag = rosbag.Bag('/home/brik/Desktop/paper_analysis/baro_residuals_no_baro.bag')
topic='/rio/odometry_optimizer'

# Read topics
q_velocities = []
f_velocities = []
n_velocities = []
pressure = []
start = 1705255794

duration = 4 # window size
for topic, msg, t in q_bag.read_messages(topics=[topic]):
    if msg.header.stamp.to_sec() > start and msg.header.stamp.to_sec() < start+duration:
        q_velocities.append([msg.header.stamp.to_sec() - start-1, msg.twist.twist.linear.z])
for topic, msg, t in f_bag.read_messages(topics=[topic]):
    if msg.header.stamp.to_sec() > start and msg.header.stamp.to_sec() < start+duration:
        f_velocities.append([msg.header.stamp.to_sec() - start-1, msg.twist.twist.linear.z])
for topic, msg, t in no_baro_bag.read_messages(topics=[topic]):
    if msg.header.stamp.to_sec() > start and msg.header.stamp.to_sec() < start+duration:
        n_velocities.append([msg.header.stamp.to_sec() - start-1, msg.twist.twist.linear.z])
for topic, msg, t in q_bag.read_messages(topics=['/baro/pressure']):
    if msg.header.stamp.to_sec() > start and msg.header.stamp.to_sec() < start+duration:
        pressure.append([msg.header.stamp.to_sec() - start-1, msg.fluid_pressure])
# Plot height
plt.figure(figsize=(3.5, 2))
next(plt.gca()._get_lines.prop_cycler)
next(plt.gca()._get_lines.prop_cycler)
next(plt.gca()._get_lines.prop_cycler)
plt.plot(np.array(q_velocities)[:,0], np.array(q_velocities)[:,1], marker='*',markevery=4, linewidth=1)
plt.plot(np.array(f_velocities)[:,0], np.array(f_velocities)[:,1], marker='o',markevery=4, linewidth=1)
next(plt.gca()._get_lines.prop_cycler)
next(plt.gca()._get_lines.prop_cycler)
next(plt.gca()._get_lines.prop_cycler)
plt.plot(np.array(n_velocities)[:,0], np.array(n_velocities)[:,1], label=r'\textrm{Radar only}', marker='^',markevery=4, linewidth=1)
plt.xlabel(r'$\displaystyle \textrm{Time }[s]$')
plt.ylabel(r'$\displaystyle {_I\mathbf{e}}_z^T \cdot {_I\mathbf{v}}_{IB}^i ~ [ms^{-1}]$')
plt.ylim(-0.11, 0.06)
lines_1, labels_1 = plt.gca().get_legend_handles_labels()
#plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.3),
#        ncol=3, fancybox=False, shadow=False, prop={'size': 8})
plt.right_ax = plt.gca().twinx()
plt.right_ax.plot(np.array(pressure)[:,0], np.array(pressure)[:,1], label=r'\textrm{Pressure}', color='grey', linewidth=0.5)
lines_2, labels_2 = plt.right_ax.get_legend_handles_labels()
plt.ylabel(r'$\displaystyle p^i ~ [\textrm{Pa}]$')
lines = lines_1 + lines_2
labels = labels_1 + labels_2
plt.legend(lines, labels, loc='upper left',prop={'size': 8})
#plt.gca().set_yscale('log')
plt.grid()
plt.xlim(0, 2)
#plt.show()

plt.tight_layout()
plt.savefig(f'/home/brik/Desktop/paper_analysis/robust_landing.pdf', bbox_inches='tight')