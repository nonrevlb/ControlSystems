# Inverted Pendulum Parameter File
import numpy as np
import control as cnt
from scipy.signal import place_poles as place

# Physical parameters of the inverted pendulum
mc = 1500.0
m = 500.0
b = 200.0
L = 10.0
g = 9.81

# Initial Conditions
z0 = 0.0                # ,m
zdot0 = 0.0             # ,m
theta0 = 0.0*np.pi/180  # ,rads
thetadot0 = 0.0*np.pi/180   # ,rads
F0 = 0.0            # ,N

# Simulation Parameters
Ts = 0.01
sigma = 0.05


####################################################
#                 State Space
####################################################
F_max = 20000                    # Max Force, N
error_max = 1                # Max step size,m
theta_max = 80.0*np.pi/180.0 # Max theta
M = 10                        # Time scale separation between
					         # inner and outer loop

# State Space Equations
# xdot = A*x + B*u
# y = C*x
A = np.matrix([[-b/mc, 0.0,               0.0,      m*g/mc],
			   [b/(mc*L), 0.0,               0.0,      -(mc+m)*g/(mc*L)],
			   [1.0,  0.0,		        0.0,      0.0],
			   [0.0, 1.0, 0.0, 0.0]])

B = np.matrix([[1.0/mc],
			   [-1.0/mc/L],
			   [0.0],
			   [0.0]])

C = np.matrix([[0.0,0.0,1.0,0.0],
				[0.0,0.0,0.0,1.0]])

Cr = C[0,:]

x0 = np.matrix([[0.0],
			   [0.0],
			   [0.0],
			   [0.0]])

# Desired Closed Loop tuning parameters
# S**2 + 2*zeta*wn*S + wn**2

z_tr = 2.4
z_zeta = .7
th_tr = 2.4
th_zeta = .6

th_wn = 2.2/th_tr
z_wn = 2.2/z_tr

# S**2 + alpha1*S + alpha0
th_alpha1 = 2.0*th_zeta*th_wn
th_alpha0 = th_wn**2

# S**2 + alpha1*S + alpha0
z_alpha1 = 2.0*z_zeta*z_wn
z_alpha0 = z_wn**2

# Desired Poles
des_char_poly = np.convolve([1,z_alpha1,z_alpha0],[1,th_alpha1,th_alpha0])
des_poles = np.roots(des_char_poly)

print des_poles

# Controllability Matrix
if np.linalg.matrix_rank(cnt.ctrb(A,B))!=4:
	print("The system is not controllable")
else:
	K = cnt.acker(A,B,des_poles)
	kr = -1.0/(Cr*np.linalg.inv(A-B*K)*B)

print('K: ', K)
print('kr: ', kr)

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
	Lo = place(A.T,C.T,obs_des_poles).gain_matrix.T
	print('L:', Lo)
