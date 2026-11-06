import numpy as np

def rad2steps(angle):
    return int(round((angle + np.pi) / (2 * np.pi) * 4095))

def steps2rad(steps):
    return steps / 4095 * 2 * np.pi - np.pi