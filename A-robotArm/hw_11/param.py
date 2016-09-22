# Inverted Pendulum Parameter File
import numpy as np
import control as cnt

# Physical parameters of the inverted pendulum
m = 0.5    # Mass of arm, kg
ell = 0.3  # Length of arm, m
b = 0.01   # Damping coefficient, Nms
g = 9.8    # Gravity coefficient, m/s**2

# Simulation Parameters
Ts = 0.01 
sigma = 0.05

# Initial Conditions
theta0 = 0.0*np.pi/180 # Initial theta position, rads
thetadot0 = 0.0  

# Equilibrium tau at theta = 0 
theta_e = 0.0*np.pi/180.0
tau_e = m*g*ell*np.cos(theta_e)/2.0 
tau_max = 1                     # Max torque input, Nm 

####################################################
#                    State Space
####################################################
# Open Loop
# b0/(S**2 + a1*S + a0)
b0 = 3/(m*ell**2)
a1 = 3*b/(m*ell**2)
a0 = 0.0

# Desired Closed Loop tuning parameters
# S**2 + 2*zeta*wn*S + wn**2

tr = 0.4          # Rise time, s
zeta = 0.707      # Damping Coefficient
wn = 2.2/tr       # Natural frequency

# S**2 + alpha1*S + alpha0
alpha1 = 2*zeta*wn
alpha0 = wn**2 

# State Space Equations
# xdot = A*x + B*u
# y = C*x
A = np.matrix([[0.0,1.0],
			   [-a0,-a1]])

B = np.matrix([[0.0],
			   [b0]])

C = np.matrix([[1.0,0.0]])

# Desired Poles
des_char_poly = [1,alpha1,alpha0]
des_poles = np.roots(des_char_poly)

# Controllability Matrix
if np.linalg.matrix_rank(cnt.ctrb(A,B))!=2:
	print("The system is not controllable")
else:
	K = cnt.acker(A,B,des_poles)
	kr = -1.0/(C*np.linalg.inv(A-B*K)*B)

# print('K: ', K)
# print('kr: ', kr)



####################################################
#          Uncertainty Parameters
####################################################
UNCERTAINTY_PARAMETERS = False
if UNCERTAINTY_PARAMETERS:
	alpha = 0.2;                                  # uncertainty parameter
	m = 0.5*(1+2*alpha*np.random.rand()-alpha)    # Mass of arm, kg
	ell = 0.3*(1+2*alpha*np.random.rand()-alpha)  # Length of arm, m
	b = 0.01*(1+2*alpha*np.random.rand()-alpha)   # Damping coefficient, Nms