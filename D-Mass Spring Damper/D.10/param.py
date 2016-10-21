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

kp = 2
kd = 6.5
ki = 1.6

# Simulation Parameters
Ts = 0.01
sigma = 0.05

# Initial Conditions
z0 = 0.0                # ,m
zdot0 = 0.0             # ,m
F0 = k*z0                # ,N
F_max = 5

#################################################
#          Uncertainty Parameters
#################################################
UNCERTAINTY_PARAMETERS = True
if UNCERTAINTY_PARAMETERS:
	alpha = 0.2;                                  # Uncertainty parameter
	m = m*(1+2*alpha*np.random.rand()-alpha)  # Mass of the pendulum, kg
	k =  k*(1+2*alpha*np.random.rand()-alpha)   # Length of the rod, m
	b = b*(1+2*alpha*np.random.rand()-alpha)   # Damping coefficient, Ns
