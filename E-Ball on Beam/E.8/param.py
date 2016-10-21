# Inverted Pendulum Parameter File
import numpy as np

# Physical parameters of the inverted pendulum
m1 = 0.35     # Mass of the ball, kg
m2 = 2      # Mass of the beam, kg
l =  0.5    # Length of the beam, m
r = 0.03       # Radius of the ball, m
g = 9.8       # Gravity, m/s**2

# Initial Conditions
z0 = .1                # ,m
zdot0 = 0.0             # ,m
theta0 = 0.0*np.pi/180  # ,rads
thetadot0 = 0.0*np.pi/180   # ,rads
F0 = m1*g/l*z0+m2*g/2             # ,N

# Simulation Parameters
Ts = 0.01
sigma = 0.01

z_tr = 1.8
z_zeta = .707
th_tr = z_tr/10.0
th_zeta = .707

th_wn = 2.2/th_tr
z_wn = 2.2/z_tr

A = m2*l*l/3+m1*z0*z0

th_kp = A*th_wn**2/l
th_kd = 2*A*th_zeta*th_wn/l
z_kp = -z_wn**2/g
z_kd = -2*z_zeta*z_wn/g

theta_max = 100
F_max = 15
