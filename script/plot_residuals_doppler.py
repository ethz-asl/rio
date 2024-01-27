#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
plt.rcParams['text.usetex'] = True
plt.rc('text.latex', preamble=r'\usepackage{amsmath}')
import rosbag
from rospy import Time
from rio.msg import DopplerResidual
import scipy.stats as stats


# fair loss rho, influence and weight function
def quadratic_loss(x):
    return 0.5 * x ** 2

def quadratic_weight(x):
    return np.ones_like(x)

def fair_loss(x, c):
    return c ** 2 * (np.abs(x) / c - np.log(1 + np.abs(x) / c))

def fair_influence(x, c):
    return x / (1 + np.abs(x)/c)

def fair_weight(x, c):
    return 1 / (1 + np.abs(x) / c)

# huber loss rho, influence and weight function
def huber_loss(x, k):
    return np.where(np.abs(x) < k, 0.5 * x ** 2, k * (np.abs(x) - 0.5 * k))

def huber_influence(x, k):
    return np.where(np.abs(x) < k, x, k * np.sign(x))

def huber_weight(x, k):
    return np.where(np.abs(x) < k, 1, k / np.abs(x))

# Plot cauchy loss rho, influence and weight function
def cauchy_loss(x, c):
    return 0.5 * c ** 2 * np.log(1 + (x / c) ** 2)

def cauchy_influence(x, c):
    return x / (1 + (x / c) ** 2)

def cauchy_weight(x, c):
    return 1 / (1 + (x / c) ** 2)

# Plot generalized geman-mcclure loss rho, influence and weight function
def geman_mcclure_loss(x, c):
    return 0.5 * c ** 2 * x ** 2 / (c ** 2 + x ** 2)

def geman_mcclure_influence(x, c):
    return x * c ** 4 / (c ** 2 + x ** 2) ** 2

def geman_mcclure_weight(x, c):
    return c ** 4 / (c ** 2 + x ** 2) ** 2

# Plot welsch loss rho, influence and weight function
def welsch_loss(x, c):
    return 0.5 * c ** 2 * (1 - np.exp(-(x / c) ** 2))

def welsch_influence(x, c):
    return x * np.exp(-(x / c) ** 2)

def welsch_weight(x, c):
    return np.exp(-(x / c) ** 2)

# Plot tukey loss rho, influence and weight function
def tukey_loss(x, c):
    return np.where(np.abs(x) < c, x ** 2 / 6.0 * (1 - (x / c) ** 2) ** 3, c ** 2 / 6.0)

def tukey_influence(x, c):
    return np.where(np.abs(x) < c, x * (1 - (x / c) ** 2) ** 2, 0)

def tukey_weight(x, c):
    return np.where(np.abs(x) < c, (1 - (x / c) ** 2) ** 2, 0)

# Plot DCS loss rho, influence and weight function
def dcs_loss(x, c):
    return (c ** 2 * x ** 2 + c * x ** 4) / ((x ** 2 + c) ** 2)

def dcs_influence(x, c):
    return np.where(x ** 2 > c, x * (2.0 * c / (x ** 2 + c)) ** 2, 0)

def dcs_weight(x, c):
    return np.where(x ** 2 > c, (2.0 * c / (x ** 2 + c)) ** 2, 1.0)

if __name__ == '__main__':
    # Open rosbag
    bag = rosbag.Bag('/home/brik/Desktop/paper_analysis/urban_night_flight_2024-01-14-19-14-31.bag')

    # Read topics rio/DopplerResidual
    residuals = []
    start = 1705256242.0
    duration = 10 # window size
    for topic, msg, t in bag.read_messages(topics=['/rio/doppler_residual'], start_time=Time(start), end_time=Time(start+duration)):
        residuals.append(msg.residual)

    # Read 
    start = 1705256217.0
    residuals_free = []
    for topic, msg, t in bag.read_messages(topics=['/rio/doppler_residual'], start_time=Time(start), end_time=Time(start+duration)):
        residuals_free.append(msg.residual)
    
    sigma = 0.05
    residuals = np.abs(np.array(residuals)) / sigma
    residuals_free = np.abs(np.array(residuals_free)) / sigma

    # Plot residuals
    fig, ax = plt.subplots(figsize=(3.5, 2.7))
    ax.loglog()
    #ax.hist([residuals for residuals in residuals if abs(residuals) <= 5], density=True, bins=50, color='lightblue', alpha=1.0)
    #ax.set_xscale('log')
    counts, bins, bars = ax.hist(residuals, density=True, bins=np.logspace(np.log10(0.1),np.log10(max(residuals)), 50), log=False, alpha=0.5, label=r'$\textrm{Streetcar}$')
    counts_free, bins_free, bars_free = ax.hist(residuals_free, density=True, bins=np.logspace(np.log10(0.1),np.log10(max(residuals)), 50), log=False, alpha=0.5, label=r'$\textrm{Static Scene}$')
    ax.set_xlabel(r'$\displaystyle \lVert r_{D^{i,m}} \rVert / \sigma_D$')
    ax.set_ylabel(r'\textrm{Probability Density}')
    ax.grid()
    # Save figure
    x = np.logspace(np.log10(0.1),np.log10(max(residuals)), 300)
    c_fair = 1.3998
    c_huber = 1.345
    c_cauchy = 2.3849
    c_geman_mcclure = 3.0
    c_welsch = 2.9846
    c_tukey = 4.6851
    c_dcs = 1.0
    ax_right = ax.twinx()
    ax_right.set_ylim(0, 1.05)

    ax.plot(x, 2*stats.norm.pdf(x), label='$\displaystyle \mathcal{N}(0,1)$')
    ax.set_ylim(min(counts[np.where(counts > 0)]), max(counts))
    ax_right.set_ylabel(r'$\displaystyle \rho \left( \lVert r_{D^{i,m}} \rVert / \sigma_D \right)$')
    next(ax_right._get_lines.prop_cycler)
    next(ax_right._get_lines.prop_cycler)
    next(ax_right._get_lines.prop_cycler)
    ax_right.plot(x, quadratic_weight(x), label=r'\textrm{Quadratic}', linestyle='solid', marker='*',markevery=30, linewidth=0.5)
    ax_right.plot(x, fair_weight(x, c_fair), label=r'\textrm{Fair}', linestyle='solid', marker='o',markevery=30, linewidth=0.5)
    ax_right.plot(x, huber_weight(x, c_huber), label=r'\textrm{Huber}', linestyle='solid', marker='s',markevery=30, linewidth=0.5)
    ax_right.plot(x, cauchy_weight(x, c_cauchy), label=r'\textrm{Cauchy}', linestyle='solid', marker='^',markevery=30, linewidth=0.5)
    #ax_right.plot(x, geman_mcclure_weight(x, c_geman_mcclure), label=r'\textrm{GMC}', linestyle='solid', marker='+',markevery=30, linewidth=0.5)
    ax_right.plot(x, welsch_weight(x, c_welsch), label=r'\textrm{Welsch}', linestyle='solid', marker='P',markevery=30, linewidth=0.5)
    #ax_right.plot(x, tukey_weight(x, c_tukey), label=r'\textrm{Tukey}$_{%.4f}$' % c_tukey, linestyle='dotted', marker='x',markevery=5)
    #ax_right.plot(x, dcs_weight(x, c_dcs), label=r'\textrm{DCS}$_{%.4f}$' % c_dcs, linestyle='dashdot')#, marker='|',markevery=5)
    ax_right.legend(loc='upper right',prop={'size': 8})
    ax_right.legend(loc='upper center', bbox_to_anchor=(0.5, 1.3),
          ncol=3, fancybox=False, shadow=False, prop={'size': 8})
    ax.legend(loc='lower left',prop={'size': 8})
    #plt.show()
    plt.tight_layout()
    plt.savefig(f'/home/brik/Desktop/paper_analysis/residuals_doppler.pdf', bbox_inches='tight')
    