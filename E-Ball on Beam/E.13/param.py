# Inverted Pendulum Parameter File
import numpy as np
from scipy.signal import place_poles as place
import control as cnt

# Physical parameters of the inverted pendulum
m1 = 0.35     # Mass of the ball, kg
m2 = 2      # Mass of the beam, kg
l =  0.5    # Length of the beam, m
r = 0.03       # Radius of the ball, m
g = 9.8       # Gravity, m/s**2

# Initial Conditions
z0 =   0.0              # ,m
zdot0 = 0.0             # ,m
theta0 = 0.0*np.pi/180  # ,rads
thetadot0 = 0.0*np.pi/180   # ,rads
F0 = m1*g/l*z0+m2*g/2             # ,N
b1 = m1*g/l
b2 = m2*g/2

# Simulation Parameters
Ts = 0.01
sigma = 0.05
input_disturbance = 0.5

a = m2*l**2/3+m1*z0**2

####################################################
#                 State Space
####################################################
F_max = 15                    # Max Force, N
error_max = 1                # Max step size,m
theta_max = 80.0*np.pi/180.0 # Max theta
M = 10                        # Time scale separation between
					         # inner and outer loop

# State Space Equations
# xdot = A*x + B*u
# y = C*x
A = np.matrix([[0.0, 0.0,               1.0,      0.0],
			   [0.0, 0.0,               0.0,      1.0],
			   [0.0,  -g,		        0.0,      0.0],
			   [-m1*g/a, 0.0, 0.0, 0.0]])

B = np.matrix([[0.0],
			   [0.0],
			   [0.0],
			   [l/a]])

C = np.matrix([[1.0,0.0,0.0,0.0],
			   [0.0,1.0,0.0,0.0]])

Cr = C[0,:]

x0 = np.matrix([[z0],
			   [theta0],
			   [zdot0],
			   [thetadot0]])

# Augmented Matrices
A1 = np.concatenate((
	np.concatenate((A,np.zeros((4,1))),axis=1),
	np.concatenate((-Cr,np.matrix([[0.0]])),axis=1)),axis = 0)



B1 = np.concatenate((B,np.matrix([[0.0]])),axis = 0)

# Desired Closed Loop tuning parameters
# S**2 + 2*zeta*wn*S + wn**2

z_tr = 1.0
z_zeta = .707
th_tr = 1.0
th_zeta = .5

integrator_pole = -20

th_wn = 2.2/th_tr
z_wn = 2.2/z_tr

# S**2 + alpha1*S + alpha0
th_alpha1 = 2.0*th_zeta*th_wn
th_alpha0 = th_wn**2

# S**2 + alpha1*S + alpha0
z_alpha1 = 2.0*z_zeta*z_wn
z_alpha0 = z_wn**2

# Desired Poles
des_char_poly = np.convolve(np.convolve([1,z_alpha1,z_alpha0],[1,th_alpha1,th_alpha0]),np.poly(integrator_pole))
des_poles = np.roots(des_char_poly)

# Controllability Matr707ix
if np.linalg.matrix_rank(cnt.ctrb(A1,B1))!=5:
	print("The system is not controllable")
else:
	K1 = cnt.acker(A1,B1,des_poles)
	K = K1[0,0:4]
	ki = K1[0,4]

print('K: ', K)
print('ki: ', ki)

####################################################
#                 Observer
####################################################

# Observer design
obs_th_wn = 5.0*th_wn
obs_z_wn = 5.0*z_wn
obs_des_char_poly = np.convolve([1,2.0*z_zeta*obs_z_wn,obs_z_wn**2],
								 [1,2.0*th_zeta*obs_th_wn,obs_th_wn**2])
obs_des_poles = np.roots(obs_des_char_poly)

if np.linalg.matrix_rank(cnt.obsv(A,C))!=4:
	print('System Not Observable')
else:
	L = place(A.T,C.T,obs_des_poles).gain_matrix.T
	print('L:', L)

#################################################
#          Uncertainty Parameters
#################################################
UNCERTAINTY_PARAMETERS = True
if UNCERTAINTY_PARAMETERS:
	alpha = 0.0                                   # Uncertainty parameter
	m1 = m1*(1+2*alpha*np.random.rand()-alpha)  # Mass of the pendulum, kg
	m2 =  m2*(1+2*alpha*np.random.rand()-alpha)   # Length of the rod, m
	l = l*(1+2*alpha*np.random.rand()-alpha)   # Damping coefficient, Ns
