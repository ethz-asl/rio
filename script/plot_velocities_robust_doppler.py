#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
plt.rcParams['text.usetex'] = True
plt.rc('text.latex', preamble=r'\usepackage{amsmath}')
import rosbag
from rospy import Time
from nav_msgs.msg import Odometry

# Open rosbag
q_bag = rosbag.Bag('/home/brik/Desktop/paper_analysis/robust_hovering/quadratic.bag')
f_bag = rosbag.Bag('/home/brik/Desktop/paper_analysis/robust_hovering/fair.bag')
h_bag = rosbag.Bag('/home/brik/Desktop/paper_analysis/robust_hovering/huber.bag')
c_bag = rosbag.Bag('/home/brik/Desktop/paper_analysis/robust_hovering/cauchy.bag')
w_bag = rosbag.Bag('/home/brik/Desktop/paper_analysis/robust_hovering/welsch.bag')
topic='/rio/odometry_optimizer'

# Read topics
q_velocities = []
f_velocities = []
h_velocities = []
c_velocities = []
w_velocities = []
start = 1705256241.5
duration = 12 # window size
for topic, msg, t in q_bag.read_messages(topics=[topic]):
    if msg.header.stamp.to_sec() > start and msg.header.stamp.to_sec() < start+duration:
        q_velocities.append([msg.header.stamp.to_sec() - start-1, np.linalg.norm([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])])
for topic, msg, t in f_bag.read_messages(topics=[topic]):
    if msg.header.stamp.to_sec() > start and msg.header.stamp.to_sec() < start+duration:
        f_velocities.append([msg.header.stamp.to_sec() - start-1, np.linalg.norm([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])])
for topic, msg, t in h_bag.read_messages(topics=[topic]):
    if msg.header.stamp.to_sec() > start and msg.header.stamp.to_sec() < start+duration:
        h_velocities.append([msg.header.stamp.to_sec() - start-1, np.linalg.norm([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])])
for topic, msg, t in c_bag.read_messages(topics=[topic]):
    if msg.header.stamp.to_sec() > start and msg.header.stamp.to_sec() < start+duration:
        c_velocities.append([msg.header.stamp.to_sec() - start-1, np.linalg.norm([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])])
for topic, msg, t in w_bag.read_messages(topics=[topic]):
    if msg.header.stamp.to_sec() > start and msg.header.stamp.to_sec() < start+duration:
        w_velocities.append([msg.header.stamp.to_sec() - start-1, np.linalg.norm([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])])
# Plot velocities
plt.figure(figsize=(3.5, 1.6))
next(plt.gca()._get_lines.prop_cycler)
next(plt.gca()._get_lines.prop_cycler)
next(plt.gca()._get_lines.prop_cycler)
plt.plot(np.array(q_velocities)[:,0], np.array(q_velocities)[:,1], label=r'\textrm{Quadratic}', marker='*',markevery=8, linewidth=1)
plt.plot(np.array(f_velocities)[:,0], np.array(f_velocities)[:,1], label=r'\textrm{Fair}', marker='o',markevery=8, linewidth=1)
plt.plot(np.array(h_velocities)[:,0], np.array(h_velocities)[:,1], label=r'\textrm{Huber}', marker='s',markevery=8, linewidth=1)
plt.plot(np.array(c_velocities)[:,0], np.array(c_velocities)[:,1], label=r'\textrm{Cauchy}', marker='^',markevery=8, linewidth=1)
plt.plot(np.array(w_velocities)[:,0], np.array(w_velocities)[:,1], label=r'\textrm{Welsch}', marker='P',markevery=8, linewidth=1)
plt.xlabel(r'$\displaystyle \textrm{Time }[s]$')
plt.ylabel(r'$\displaystyle \lVert {_I\mathbf{v}_{IB}} \rVert ~ [ms^{-1}]$')
plt.gca().set_yscale('log')
#plt.legend(loc='lower right',prop={'size': 8})
plt.grid()
plt.xlim(0, 10)
#plt.show()

plt.tight_layout()
plt.savefig(f'/home/brik/Desktop/paper_analysis/robust_hovering.pdf', bbox_inches='tight')