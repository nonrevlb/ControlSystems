# Inverted Pendulum Parameter File
import numpy as np

# Physical parameters of the inverted pendulum
mc = 0.35   # Mass of the center pod, kg
mr = 2      # Mass of right rotor, kg
ml = 2      # Mass of left rotor, kg
Jc = .0042  # Inertia of center pod, kg*m^2
d =  0.3    # Distance to each rotor, m
wc = 0.1    # Width of center pod, m
hc = 0.1    # Height of center pod, m
r = 0.03    # Radius of each rotor, m
wt = 0.15   # Width of target, m
ht = 0.03   # Height of target, m
mu = 0.1    # Drag constant, kg/s
g = 9.8     # Gravity, m/s**2

# Simulation Parameters
Ts = 0.01

# Initial Conditions
zv0 = 2.0                # ,m
h0 = 2.0
zt0 = 2.0
theta0 = 0.0*np.pi/180  # ,rads
