# Inverted Pendulum Parameter File
import numpy as np

# Physical parameters of the inverted pendulum
m1e = 0.35     # Mass of the ball, kg
m2e = 2      # Mass of the beam, kg
le =  0.5    # Length of the beam, m
r = 0.03       # Radius of the ball, m
g = 9.8       # Gravity, m/s**2

# Initial Conditions
z0 = .1                # ,m
zdot0 = 0.0             # ,m
theta0 = 0.0*np.pi/180  # ,rads
thetadot0 = 0.0*np.pi/180   # ,rads
F0 = m1e*g/le*z0+m2e*g/2             # ,N

# Simulation Parameters
Ts = 0.01
sigma = 0.05

z_tr = 2.0
z_zeta = .707
th_tr = z_tr/10.0
th_zeta = .707

th_wn = 2.2/th_tr
z_wn = 2.2/z_tr

A = m2e*le*le/3+m1e*z0*z0

th_kp = A*th_wn**2/le
th_kd = 2*A*th_zeta*th_wn/le
th_ki = 0
z_kp = -z_wn**2/g
z_kd = -2*z_zeta*z_wn/g
z_ki = -.1

theta_max = 90*np.pi/180
F_max = 15

#################################################
#          Uncertainty Parameters
#################################################
UNCERTAINTY_PARAMETERS = True
if UNCERTAINTY_PARAMETERS:
	alpha = 0.1                                   # Uncertainty parameter
	m1 = m1e*(1+2*alpha*np.random.rand()-alpha)  # Mass of the pendulum, kg
	m2 =  m2e*(1+2*alpha*np.random.rand()-alpha)   # Length of the rod, m
	l = le*(1+2*alpha*np.random.rand()-alpha)   # Damping coefficient, Ns
else:
    m1 = m1e
    m2 = m2e
    l = le
