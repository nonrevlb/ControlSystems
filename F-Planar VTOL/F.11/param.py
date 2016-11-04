# Inverted Pendulum Parameter File
import numpy as np
import control as cnt

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

de = d

M = mc + mr + ml
I = Jc + mr*d**2.0 + ml*d**2.0

# Simulation Parameters
Ts = 0.01
sigma = 0.05

#h control variables
h_tr = 3.0
h_wn = 2.2/h_tr
h_zeta = .707

#z control variables
z_tr = 3.0
z_wn = 2.2/z_tr
z_zeta = .707

#theta control variables
th_tr = z_tr/10
th_wn = 2.2/th_tr
th_zeta = 0.707

# Initial Conditions
zv0 = 0.5               # ,m
zvdot0 = 0.0
h0 = 1.0
hdot0 = 0.0
theta0 = 0.0*np.pi/180  # ,rads
thetadot0 = 0.0
zt0 = 2.0

F0 = (mc+2.0*mr)*g
T0 = 0.0
Fr0 = (mc+2.0*mr)*g/2.0
Fl0 = (mc+2.0*mr)*g/2.0

####################################################
#                 State Space
####################################################
F_max = 10                    # Max Force, N
error_max = 1                # Max step size,m
theta_max = 90.0*np.pi/180.0 # Max theta

# State Space Equations
# xdot = A*x + B*u
# y = C*x
A_lat = np.matrix([[0.0,0.0,1.0,0.0],
			   [0.0,0.0,0.0,1.0],
			   [0.0,-g,-mu/M,0.0],
			   [0.0,0.0,0.0,0.0]])

B_lat = np.matrix([[0.0],
			   [0.0],
			   [0.0],
			   [1/I]])

C_lat = np.matrix([[1.0,0.0,0.0,0.0]])

x0_lat = np.matrix([[zv0],
			   [theta0],
			   [zvdot0],
			   [thetadot0]])

A_lon = np.matrix([[0.0,1.0],
				   [0.0,0.0]])

B_lon = np.matrix([[0  ],
				  [1.0/M]])

C_lon = np.matrix([[1.0,0.0]])

x0_lon = np.matrix([[h0],
					[hdot0]])

# S**2 + alpha1*S + alpha0
th_alpha1 = 2.0*th_zeta*th_wn
th_alpha0 = th_wn**2

# S**2 + alpha1*S + alpha0
z_alpha1 = 2.0*z_zeta*z_wn
z_alpha0 = z_wn**2

# S**2 + alpha1*S + alpha0
h_alpha1 = 2.0*h_zeta*h_wn
h_alpha0 = h_wn**2

# Desired Poles
des_char_poly_lat = np.convolve([1,z_alpha1,z_alpha0],[1,th_alpha1,th_alpha0])
des_char_poly_lon = [1,h_alpha1,h_alpha0]
des_poles_lat = np.roots(des_char_poly_lat)
des_poles_lon = np.roots(des_char_poly_lon)

# Latitudinal Controllability Matrix
if np.linalg.matrix_rank(cnt.ctrb(A_lat,B_lat))!=4:
	print("The Latitudinal system is not controllable")
else:
	K_lat = cnt.acker(A_lat,B_lat,des_poles_lat)
	kr_lat = -1.0/(C_lat*np.linalg.inv(A_lat-B_lat*K_lat)*B_lat)

# Longitudinal Controllability Matrix
if np.linalg.matrix_rank(cnt.ctrb(A_lon,B_lon))!=2:
	print("The Longitudinal system is not controllable")
else:
	K_lon = cnt.acker(A_lon,B_lon,des_poles_lon)
	kr_lon = -1.0/(C_lon*np.linalg.inv(A_lon-B_lon*K_lon)*B_lon)

print('K_lat: ', K_lat)
print('kr_lat: ', kr_lat)
print('K_lon: ', K_lon)
print('kr_lon: ', kr_lon)

#################################################
#          Uncertainty Parameters
#################################################
UNCERTAINTY_PARAMETERS = True
if UNCERTAINTY_PARAMETERS:
	alpha = 0.0                                   # Uncertainty parameter
	mc = mc*(1+2*alpha*np.random.rand()-alpha)  # Mass of the pendulum, kg
	Jc = Jc*(1+2*alpha*np.random.rand()-alpha)   # Length of the rod, m
	d = d*(1+2*alpha*np.random.rand()-alpha)   # Damping coefficient, Ns
	mu = mu*(1+2*alpha*np.random.rand()-alpha)   # Damping coefficient, Ns
