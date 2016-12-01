# Inverted Pendulum Parameter File
import numpy as np
import control as cnt

# Physical parameters of the inverted pendulum
m = 5.0     # Mass of the block, kg
k =  3.0    # Spring constant, kg/s^2
l = 2.0     # Rest length of spring, m
b = 0.5     # Damping canstant, kg/s
w = 0.5       # Width of the block, m
h = 0.2      # Height of the block, m
g = 9.8       # Gravity, m/s**2

# Simulation Parameters
Ts = 0.01
sigma = 0.05
input_disturbance = 0.0

# Initial Conditions
z0 = 0.0                # ,m
zdot0 = 0.0             # ,m
F0 = k*z0                # ,N

####################################################
#                 State Space
####################################################
F_max = 5                    # Max Force, N
error_max = 1                # Max step size,m
theta_max = 30.0*np.pi/180.0 # Max theta
M = 3                        # Time scale separation between

# State Space Equations
# xdot = A*x + B*u
# y = C*x
A = np.matrix([[0.0,1.0],
			   [-k/m,-b/m]])

B = np.matrix([[0.0],
			   [1/m]])

C = np.matrix([[1.0,0.0]])

A1 = np.concatenate((
	np.concatenate((A,np.zeros((2,1))),axis=1),
	np.concatenate((-C,np.matrix([[0.0]])),axis=1)),axis = 0)

B1 = np.concatenate((B,np.matrix([[0.0]])),axis = 0)

# Desired Closed Loop tuning parameters
# S**2 + 2*zeta*wn*S + wn**2

z_tr = 2.0      # Rise time, s
z_zeta = 0.707      # Damping Coefficient
z_wn = 2.2/z_tr     # Natural frequency

integrator_pole = -5


# S**2 + alpha1*S + alpha0
z_alpha1 = 2.0*z_zeta*z_wn
z_alpha0 = z_wn**2

# Desired Poles
des_char_poly = np.convolve([1,z_alpha1,z_alpha0],np.poly(integrator_pole))
des_poles = np.roots(des_char_poly)

# Controllability Matrix
if np.linalg.matrix_rank(cnt.ctrb(A1,B1))!=3:
	print("The system is not controllable")
else:
	K1 = cnt.acker(A1,B1,des_poles)
	K = K1[0,0:2]
	ki = K1[0,2]

print('K: ', K)
print('ki: ', ki)

####################################################
#                 Observer
####################################################

# Observer design
obs_z_wn = 5.0*z_wn
obs_des_char_poly = [1,2.0*z_zeta*obs_z_wn,obs_z_wn**2]
obs_des_poles = np.roots(obs_des_char_poly)


if np.linalg.matrix_rank(cnt.obsv(A,C))!=2:
	print('System Not Observable')
else:
	L = cnt.acker(A.T,C.T,obs_des_poles).T
	print('L:', L)

#################################################
#          Uncertainty Parameters
#################################################
UNCERTAINTY_PARAMETERS = True
if UNCERTAINTY_PARAMETERS:
	alpha = 0.0;                                  # Uncertainty parameter
	m = m*(1+2*alpha*np.random.rand()-alpha)  # Mass of the pendulum, kg
	k =  k*(1+2*alpha*np.random.rand()-alpha)   # Length of the rod, m
	b = b*(1+2*alpha*np.random.rand()-alpha)   # Damping coefficient, Ns
