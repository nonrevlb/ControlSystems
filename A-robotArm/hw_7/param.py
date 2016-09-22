# Inverted Pendulum Parameter File
import numpy as np

# Physical parameters of the inverted pendulum
m = 0.5   # Mass of arm, kg
ell = 0.3   # Length of arm, m
b = 0.01  # Damping coefficient, Nms
g = 9.8   # Gravity coefficient, m/s**2

# Simulation Parameters
Ts = 0.01 
sigma = 0.05

# Initial Conditions
theta0 = 0.0*np.pi/180 # Initial theta position, rads
thetadot0 = 0.0  

# Equilibrium tau at theta = 0 
tau_e = m*g*ell/2  

####################################################
#    PD Control: Pole Placement
####################################################

# Open Loop
# b0/(S**2 + a1*S + a0)
b0 = 3/(m*ell**2)
a1 = 3*b/(m*ell**2)
a0 = 0

# Desired Poles 
p1 = -3.0  
p2 = -4.0

# Desired Closed Loop
# S**2 + alpha1*S + alpha0
CL = np.polynomial.polynomial.polyfromroots((p1,p2))
alpha1 = CL[1]
alpha0 = CL[0] 

# Gains
# b0*kp/[S**2 + (a1 + b0*kd)S + (a0 + b0*kp)]
kp = (alpha0-a0)/b0
kd = (alpha1-a1)/b0

print('kp: ',kp)
print('kd: ',kd)
