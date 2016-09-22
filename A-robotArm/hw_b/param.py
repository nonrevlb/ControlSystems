# Inverted Pendulum Parameter File
import numpy as np

# Physical parameters of the inverted pendulum
m = 0.5   # Mass of arm, kg
ell = 0.3   # Length of arm, m
b = 0.01  # Damping coefficient, Nms
g = 9.8   # Gravity coefficient, m/s**2

# Simulation Parameters
Ts = 0.01 

# Initial Conditions
theta0 = 0.0*np.pi/180 # Initial theta position, rads
thetadot0 = 0.0  

tau_e = m*g*ell/2      # Equilibrium tau    
