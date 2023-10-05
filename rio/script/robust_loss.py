

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

if __name__ == '__main__':
    # Plot huber and cauchy in same 3,1 subplots
    fig, ax = plt.subplots(3, 1, figsize=(8, 8))
    x = np.linspace(-3, 3, 100)
    k = 0.1
    c = 0.1
    ax[0].plot(x, fair_loss(x, k), label='Fair loss')
    ax[0].plot(x, huber_loss(x, k), label='Huber loss')
    ax[0].plot(x, cauchy_loss(x, c), label='Cauchy loss')
    ax[0].legend()
    ax[0].set_title('Loss function')
    ax[0].grid()
    ax[1].plot(x, fair_influence(x, k), label='Fair influence')
    ax[1].plot(x, huber_influence(x, k), label='Huber influence')
    ax[1].plot(x, cauchy_influence(x, c), label='Cauchy influence')
    ax[1].legend()
    ax[1].set_title('Influence function')
    ax[1].grid()
    ax[2].plot(x, fair_weight(x, k), label='Fair weight')
    ax[2].plot(x, huber_weight(x, k), label='Huber weight')
    ax[2].plot(x, cauchy_weight(x, c), label='Cauchy weight')
    ax[2].legend()
    ax[2].set_title('Weight function')
    ax[2].grid()
    plt.tight_layout()
    plt.show()
