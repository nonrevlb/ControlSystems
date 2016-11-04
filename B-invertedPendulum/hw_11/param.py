# Inverted Pendulum Parameter File
import numpy as np
import control as cnt

# Physical parameters of the inverted pendulum
m1 = 0.25     # Mass of the pendulum, kg
m2 = 1.0      # Mass of the cart, kg
ell =  0.5    # Length of the rod, m
w = 0.5       # Width of the cart, m
h = 0.15      # Height of the cart, m
gap = 0.005   # Gap between the cart and x-axis
radius = 0.06 # Radius of circular part of pendulum
g = 9.8       # Gravity, m/s**2
b = 0.05      # Damping coefficient, Ns

# Simulation Parameters
Ts = 0.01 
sigma = 0.05

# Initial Conditions
z0 = 0.0                # ,m
theta0 = 0.0*np.pi/180  # ,rads
zdot0 = 0.0             # ,m/s
thetadot0 = 0.0         # ,rads/s

####################################################
#                 State Space
####################################################
F_max = 5                    # Max Force, N
error_max = 1                # Max step size,m
theta_max = 30.0*np.pi/180.0 # Max theta 
M = 3                        # Time scale separation between
					         # inner and outer loop

# State Space Equations
# xdot = A*x + B*u
# y = C*x
A = np.matrix([[0.0,0.0,               1.0,      0.0],
			   [0.0,0.0,               0.0,      1.0],
			   [0.0,-m1*g/m2,          b/m2,     0.0],
			   [0.0,(m1+m2)*g/m2/ell, b/m2/ell, 0.0]])

B = np.matrix([[0.0],
			   [0.0],
			   [1.0/m2],
			   [-1.0/m2/ell]])

C = np.matrix([[1.0,0.0,0.0,0.0],
			   [0.0,1.0,0.0,0.0]])

# Desired Closed Loop tuning parameters
# S**2 + 2*zeta*wn*S + wn**2

th_tr = 0.5           # Rise time, s
th_zeta = 0.707       # Damping Coefficient
th_wn = 2.2/th_tr     # Natural frequency

# S**2 + alpha1*S + alpha0
th_alpha1 = 2.0*th_zeta*th_wn
th_alpha0 = th_wn**2 


# Desired Closed Loop tuning parameters
# S**2 + 2*zeta*wn*S + wn**2

z_tr = th_tr*M      # Rise time, s
z_zeta = 0.707      # Damping Coefficient
z_wn = 2.2/z_tr     # Natural frequency


# S**2 + alpha1*S + alpha0
z_alpha1 = 2.0*z_zeta*z_wn
z_alpha0 = z_wn**2 

# Desired Poles
des_char_poly = np.convolve([1,z_alpha1,z_alpha0],[1,th_alpha1,th_alpha0])
des_poles = np.roots(des_char_poly)

# Controllability Matrix
if np.linalg.matrix_rank(cnt.ctrb(A,B))!=4:
	print("The system is not controllable")
else:
	K = cnt.acker(A,B,des_poles)
	kr = -1.0/(C[0]*np.linalg.inv(A-B*K)*B)

# print('K: ', K)
# print('kr: ', kr)


#################################################
#          Uncertainty Parameters
#################################################
UNCERTAINTY_PARAMETERS = False
if UNCERTAINTY_PARAMETERS:
	alpha = 0.2;                                  # Uncertainty parameter
	m1 = 0.25*(1+2*alpha*np.random.rand()-alpha)  # Mass of the pendulum, kg
	m2 = 1.0*(1+2*alpha*np.random.rand()-alpha)   # Mass of the cart, kg
	L =  0.5*(1+2*alpha*np.random.rand()-alpha)   # Length of the rod, m
	b = 0.05*(1+2*alpha*np.random.rand()-alpha)   # Damping coefficient, Ns


