# Inverted Pendulum Parameter File
import numpy as np

# Physical parameters of the inverted pendulum
m1 = 0.35     # Mass of the ball, kg
m2 = 2      # Mass of the beam, kg
l =  0.5    # Length of the beam, m
r = 0.03       # Radius of the ball, m
g = 9.8       # Gravity, m/s**2

# Initial Conditions
z0 = l/2                # ,m
zdot0 = 0.0             # ,m
theta0 = 0.0*np.pi/180  # ,rads
thetadot0 = 0.0*np.pi/180   # ,rads
F0 = m1*g/l*z0+m2*g/2             # ,N

# Simulation Parameters
Ts = 0.01
sigma = 0.01

# th_tr = 1
# th_zeta = .707
# z_tr = 10
# z_zeta = .707
#
# th_wn = 2.2/th_tr
# z_Wn = 2.2/z_tr
#
# A = m2*l*l/3+m1*z0*z0

th_kp = 1.825
th_kd = 1.173
z_kp = -.0049
z_kd = -.0317

theta_max = 100
F_max = 10000
