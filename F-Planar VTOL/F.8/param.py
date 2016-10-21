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

M = mc + mr + ml
I = Jc + mr*d**2.0 + ml*d**2.0

# Simulation Parameters
Ts = 0.01
sigma = 0.5

#h control variables
h_tr = 3.6
h_wn = 2.2/h_tr
h_zeta = .707

h_kp = M*h_wn**2.0
h_kd = 2.0*M*h_zeta*h_wn
print ("h_kp", h_kp)
print ("h_kd", h_kd)

#z control variables
z_tr = 2.0
z_wn = 2.2/z_tr
z_zeta = .707

z_kp = -z_wn**2.0/g
z_kd = -(2.0*z_zeta*z_wn - mu/M)/g
print ("z_kp", z_kp)
print ("z_kd", z_kd)

#theta control variables
th_tr = z_tr/10
th_wn = 2.2/th_tr
th_zeta = .707

th_kp = I*th_wn**2.0
th_kd = 2.0*I*th_zeta*th_wn
print ("th_kp", th_kp)
print ("th_kd", th_kd)

# Initial Conditions
zv0 = 0.5               # ,m
zvdot0 = 0.0
h0 = 1.0
hdot0 = 0.0
theta0 = 0.0*np.pi/180  # ,rads
thetadot0 = 0.0
zt0 = 2.0

F0 = (mc+2.0*mr)*g
Fr0 = (mc+2.0*mr)*g/2.0
Fl0 = (mc+2.0*mr)*g/2.0

Fr_max = 10.0
Fl_max = 10.0
