# Inverted Pendulum Parameter File
import numpy as np

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
#       PD Control: Time Design Strategy
####################################################
F_max = 5             		  # Max Force, N
error_max = 1        		  # Max step size,m
theta_max = 30.0*np.pi/180.0  # Max theta, rads
M = 14 #25.0             		  # Time scale separation between
					 		  # inner and outer loop

#---------------------------------------------------
#                    Inner Loop
#---------------------------------------------------
# Open Loop
# b0/(S**2 + a1*S + a0)
th_b0 = -2.0/(m2*ell)
th_a1 = 0.0
th_a0 = -2.0*(m1+m2)*g/(m2*ell)

# Desired Closed Loop tuning parameters
# S**2 + 2*zeta*wn*S + wn**2

th_tr = 0.5 #0.45          # Rise time, s
th_zeta = 0.707       # Damping Coefficient
th_wn = 2.2/th_tr     # Natural frequency

# S**2 + alpha1*S + alpha0
th_alpha1 = 2.0*th_zeta*th_wn
th_alpha0 = th_wn**2 

# Gains
# b0*kp/[S**2 + (a1 + b0*kd)S + (a0 + b0*kp)]
th_kp = (th_alpha0-th_a0)/th_b0
th_kd = (th_alpha1-th_a1)/th_b0
th_DC = th_kp/((m1+m2)*g+th_kp)   #DC gain


#---------------------------------------------------
#                    Outer Loop
#---------------------------------------------------
# Open Loop
# b0/(S**2 + a1*S + a0)
z_b0 = (m1*g/m2)
z_a1 = b/m2
z_a0 = 0

# Desired Closed Loop tuning parameters
# S**2 + 2*zeta*wn*S + wn**2

z_tr = M*th_tr  # Rise time, s
z_zeta = 0.707      # Damping Coefficient
z_wn = 2.2/z_tr  # Natural frequency


# S**2 + alpha1*S + alpha0
z_alpha1 = 2.0*z_zeta*z_wn
z_alpha0 = z_wn**2 

# Gains
# b0*kp/[S**2 + (a1 + b0*kd*IL_DC)S + (a0 + b0*kp*IL_DC)]
z_kp = (z_alpha0-z_a0)/(th_DC*z_b0)
z_kd = (z_alpha1-z_a1)/(th_DC*z_b0)
z_DC = 1                                 # DC gain




print('th_DC', th_DC)
print('th_kp: ',th_kp)
print('th_kd: ',th_kd)
print('z_kp: ',z_kp)
print('z_kd: ',z_kd)


