# Inverted Pendulum Parameter File
import numpy as np

# Physical parameters of the inverted pendulum
m1 = 0.35     # Mass of the ball, kg
m2 = 2      # Mass of the beam, kg
l =  0.5    # Length of the beam, m
r = 0.05       # Radius of the ball, m
g = 9.8       # Gravity, m/s**2

# Simulation Parameters
Ts = 0.01

# Initial Conditions
z0 = 0.0                # ,m
zdot0 = 0.0             # ,m
theta0 = 0.0*np.pi/180  # ,rads
thetadot0 = 0.0*np.pi/180   # ,rads
F0 = m2*g/2             # ,N
