# Inverted Pendulum Parameter File
import numpy as np

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
tau_e = m*g*ell/2  


####################################################
#                    PID Control
####################################################
tau_max = 1                  # Max torque input, Nm
tau_e = m*g*ell/2            # Equilibrium torque at theta = 0, Nm

error_max = 50.0*np.pi/180.0 # Max step size

# Open Loop
# b0/(S**2 + a1*S + a0)
b0 = 3/(m*ell**2)
a1 = 3*b/(m*ell**2)
a0 = 0

# Desired Closed Loop tuning parameters
# S**2 + 2*zeta*wn*S + wn**2

tr = 0.4          # Rise time, s
zeta = 0.707      # Damping Coefficient
wn = 2.2/tr       # Natural frequency

# S**2 + alpha1*S + alpha0
alpha1 = 2*zeta*wn
alpha0 = wn**2 

# Gains
# b0*kp/[S**2 + (a1 + b0*kd)S + (a0 + b0*kp)]
kp = (alpha0-a0)/b0         # Proportional gain
kd = (alpha1-a1)/b0         # Derivative gain
ki = 0.1*0                    # Integral gain

print('kp: ',kp)
print('kd: ',kd)
print('ki: ', ki)


#################################################
#          Uncertainty Parameters
#################################################
UNCERTAINTY_PARAMETERS = True
if UNCERTAINTY_PARAMETERS:
	alpha = 0.2;                                  # uncertainty parameter
	m = 0.5*(1+2*alpha*np.random.rand()-alpha)    # Mass of arm, kg
	ell = 0.3*(1+2*alpha*np.random.rand()-alpha)  # Length of arm, m
	b = 0.01*(1+2*alpha*np.random.rand()-alpha)   # Damping coefficient, Nms