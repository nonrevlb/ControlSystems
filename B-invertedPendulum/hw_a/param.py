# Inverted Pendulum Parameter File
import numpy as np

# Physical parameters of the inverted pendulum
m1 = 0.25     # Mass of the pendulum, kg
m2 = 1.0      # Mass of the cart, kg
ell =  0.5    # Length of the rod, m
w = 0.5       # Width of the cart, m
h = 0.15      # Height of the cart, m
gap = 0.005   # Gap between the cart and x-axis
radius = 0.06 # Radius of circular part of pendulum
g = 9.8       # Gravity, m/s**2
b = 0.05      # Damping coefficient, Ns

# Simulation Parameters
Ts = 0.01 

# Initial Conditions
z0 = 0.0                # ,m
theta0 = 0.0*np.pi/180  # ,rads