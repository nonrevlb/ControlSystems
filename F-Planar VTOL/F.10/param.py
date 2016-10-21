# Inverted Pendulum Parameter File
import numpy as np

# Physical parameters of the inverted pendulum
mce = 1   # Mass of the center pod, kg
mr = .25      # Mass of right rotor, kg
ml = .25      # Mass of left rotor, kg
Jce = .0042  # Inertia of center pod, kg*m^2
de =  0.3    # Distance to each rotor, m
wc = 0.1    # Width of center pod, m
hc = 0.1    # Height of center pod, m
r = 0.03    # Radius of each rotor, m
wt = 0.15   # Width of target, m
ht = 0.03   # Height of target, m
mue = 0.1    # Drag constant, kg/s
g = 9.81     # Gravity, m/s**2

M = mce + mr + ml
I = Jce + mr*de**2.0 + ml*de**2.0

# Simulation Parameters
Ts = 0.01
sigma = 0.05

#h control variables
h_tr = 3.0
h_wn = 2.2/h_tr
h_zeta = .707

h_kp = M*h_wn**2.0
h_kd = 2.0*M*h_zeta*h_wn
h_ki = .18
print ("h_kp", h_kp)
print ("h_kd", h_kd)
print ("h_ki", h_ki)

#z control variables
z_tr = 3.0
z_wn = 2.2/z_tr
z_zeta = .707

z_kp = -z_wn**2.0/g
z_kd = -(2.0*z_zeta*z_wn - mue/M)/g
z_ki = -0.012
print ("z_kp", z_kp)
print ("z_kd", z_kd)
print ("z_ki", z_ki)

#theta control variables
th_tr = z_tr/10
th_wn = 2.2/th_tr
th_zeta = 1.707

th_kp = I*th_wn**2.0
th_kd = 2.0*I*th_zeta*th_wn
th_ki = 0
print ("th_kp", th_kp)
print ("th_kd", th_kd)
print ("th_ki", th_ki)

# Initial Conditions
zv0 = 0.5               # ,m
zvdot0 = 0.0
h0 = 1.0
hdot0 = 0.0
theta0 = 0.0*np.pi/180  # ,rads
thetadot0 = 0.0
zt0 = 2.0

F0 = (mce+2.0*mr)*g
Fr0 = (mce+2.0*mr)*g/2.0
Fl0 = (mce+2.0*mr)*g/2.0

Fr_max = 10.0
Fl_max = 10.0
theta_max = 90*np.pi/180

#################################################
#          Uncertainty Parameters
#################################################
UNCERTAINTY_PARAMETERS = True
if UNCERTAINTY_PARAMETERS:
	alpha = 0.2                                   # Uncertainty parameter
	mc = mce*(1+2*alpha*np.random.rand()-alpha)  # Mass of the pendulum, kg
	Jc = Jce*(1+2*alpha*np.random.rand()-alpha)   # Length of the rod, m
	d = de*(1+2*alpha*np.random.rand()-alpha)   # Damping coefficient, Ns
	mu = mue*(1+2*alpha*np.random.rand()-alpha)   # Damping coefficient, Ns
else:
    mc = mce
    Jc = Jce
    d = de
    mu = mue
