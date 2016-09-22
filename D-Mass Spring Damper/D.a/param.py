# Inverted Pendulum Parameter File
import numpy as np

# Physical parameters of the inverted pendulum
m = 5.0     # Mass of the block, kg
k =  3.0    # Spring constant, kg/s^2
l = 2.0     # Rest length of spring, m
b = 0.5     # Damping canstant, kg/s
w = 0.5       # Width of the block, m
h = 0.2      # Height of the block, m
g = 9.8       # Gravity, m/s**2

# Simulation Parameters
Ts = 0.01

# Initial Conditions
z0 = 0.0                # ,m
