# Inverted Pendulum Parameter File
import numpy as np

# Physical parameters of the inverted pendulum
mc = 1   # Mass of the center pod, kg
mr = .25      # Mass of right rotor, kg
ml = .25      # Mass of left rotor, kg
Jc = .0042  # Inertia of center pod, kg*m^2
d =  0.3    # Distance to each rotor, m
wc = 0.1    # Width of center pod, m
hc = 0.1    # Height of center pod, m
r = 0.03    # Radius of each rotor, m
wt = 0.15   # Width of target, m
ht = 0.03   # Height of target, m
mu = 0.1    # Drag constant, kg/s
g = 9.81     # Gravity, m/s**2

# Simulation Parameters
Ts = 0.1
sigma = 0.5

kp = .09
kd = .75

# Initial Conditions
zv0 = 2.0                # ,m
zvdot0 = 0.0
h0 = 1.0
hdot0 = 0.0
theta0 = 0.0*np.pi/180  # ,rads
thetadot0 = 0.0
zt0 = 2.0
Fr0 = (mc+2*mr)*g/2
Fl0 = (mc+2*mr)*g/2

F_max = 100.0
