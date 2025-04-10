#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

# fair loss rho, influence and weight function
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

# Plot welsh loss rho, influence and weight function
def welsh_loss(x, c):
    return 0.5 * c ** 2 * (1 - np.exp(-(x / c) ** 2))

def welsh_influence(x, c):
    return x * np.exp(-(x / c) ** 2)

def welsh_weight(x, c):
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
    # Plot huber and cauchy in same 3,1 subplots
    fig, ax = plt.subplots(3, 1, figsize=(8, 8))
    x = np.linspace(-3, 3, 100)
    sigma = 0.1
    c_fair = 1.3998 * sigma
    c_huber = 1.345 * sigma
    c_cauchy = 2.3849 * sigma
    c_geman_mcclure = 3.0 * sigma
    c_welsh = 2.9846 * sigma
    c_tukey = 4.6851 * sigma
    c_dcs = 1.0 * sigma
    ax[0].plot(x, fair_loss(x, c_fair), label='Fair loss')
    ax[0].plot(x, huber_loss(x, c_huber), label='Huber loss')
    ax[0].plot(x, cauchy_loss(x, c_cauchy), label='Cauchy loss')
    ax[0].plot(x, geman_mcclure_loss(x, c_geman_mcclure), label='Geman-McClure loss')
    ax[0].plot(x, welsh_loss(x, c_welsh), label='Welsh loss')
    ax[0].plot(x, tukey_loss(x, c_tukey), label='Tukey loss')
    ax[0].plot(x, dcs_loss(x, c_dcs), label='DCS loss')
    ax[0].legend()
    ax[0].set_title('Loss function')
    ax[0].grid()
    ax[1].plot(x, fair_influence(x, c_fair), label='Fair influence')
    ax[1].plot(x, huber_influence(x, c_huber), label='Huber influence')
    ax[1].plot(x, cauchy_influence(x, c_cauchy), label='Cauchy influence')
    ax[1].plot(x, geman_mcclure_influence(x, c_geman_mcclure), label='Geman-McClure influence')
    ax[1].plot(x, welsh_influence(x, c_welsh), label='Welsh influence')
    ax[1].plot(x, tukey_influence(x, c_tukey), label='Tukey influence')
    ax[1].plot(x, dcs_influence(x, c_dcs), label='DCS influence')
    ax[1].legend()
    ax[1].set_title('Influence function')
    ax[1].grid()
    ax[2].plot(x, fair_weight(x, c_fair), label='Fair weight')
    ax[2].plot(x, huber_weight(x, c_huber), label='Huber weight')
    ax[2].plot(x, cauchy_weight(x, c_cauchy), label='Cauchy weight')
    ax[2].plot(x, geman_mcclure_weight(x, c_geman_mcclure), label='Geman-McClure weight')
    ax[2].plot(x, welsh_weight(x, c_welsh), label='Welsh weight')
    ax[2].plot(x, tukey_weight(x, c_tukey), label='Tukey weight')
    ax[2].plot(x, dcs_weight(x, c_dcs), label='DCS weight')
    ax[2].legend()
    ax[2].set_title('Weight function')
    ax[2].grid()
    plt.tight_layout()
    plt.show()
