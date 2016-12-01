import param as P
import numpy as np

class controllerPD:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the PD_ctrl object
      self.zCtrl = zPID_ctrl(P.z_kp,P.z_kd,P.z_ki,P.z0,P.theta_max)
      self.thetaCtrl = thetaPID_ctrl(P.th_kp,P.th_kd,P.th_ki,P.z0,P.F_max)
      # kp is the proportional gain
      # kd is the derivative gain
      # y0 is the initial position of the state

  def getForces(self,y_r,y):
	  # y_r is the referenced input
	  # y is the current state
	  z_r = y_r[0]
	  z = y[0]
	  theta = y[1]
	  theta_r = self.zCtrl.zPID_loop(z_r,z)
	  F = self.thetaCtrl.thetaPID_loop(theta_r,theta,z) # Calculate the force output
	  return [F,theta_r]

class thetaPID_ctrl:
  def __init__(self,kp,kd,ki,theta0,limit):
	  self.differentiator = 0.0    # Difference term
	  self.integrator = 0.0
	  self.theta_d1 = theta0       # Delayed y output
	  self.error_d1 = 0.0          # Delayed error
	  self.kp = kp                 # Proportional control gain
	  self.kd = kd                 # Derivative control gain
	  self.ki = ki
	  self.limit = limit


  def thetaPID_loop(self,theta_r,theta,z):
	  # Compute the current error
	  error = theta_r - theta

	  # UPIDate Differentiator
	  a1 = (2*P.sigma - P.Ts)/(2*P.sigma+P.Ts)
	  a2 = 2/(2*P.sigma+P.Ts)
	  self.differentiator = a1*self.differentiator \
                          + a2*(theta -self.theta_d1)

	  # UPIDate error_d1
	  self.error_d1 = error
	  self.theta_d1 = theta

	  # Update Integrator
	  if abs(self.differentiator) <0.05:
		  self.integrator += (P.Ts/2.0)*(error+self.error_d1)

	  # PID Control to calculate T
	  Fe = P.m1e*P.g*z/P.le +0.5*P.m2e*P.g
	  F_unsat = self.kp*error - self.kd*self.differentiator + self.ki*self.integrator + Fe

	  F_sat = self.saturate(F_unsat)

	#   if self.ki != 0:
	# 	  self.integrator += P.Ts/self.ki*(F_sat-F_unsat)

	  return F_sat

  def saturate(self,u):
    if abs(u) > self.limit:
      u = self.limit*np.sign(u)
    return u

class zPID_ctrl:
  def __init__(self,kp,kd,ki,z0,limit):
      self.differentiator = 0.0    # Difference term
      self.integrator = 0.0        # Integral term
      self.z_d1 = z0               # Delayed y output
      self.error_d1 = 0.0          # Delayed error
      self.kp = kp                 # Proportional control gain
      self.kd = kd                 # Derivative control gain
      self.ki = ki                 # Integral controal gain
      self.limit = limit           # Maximum theta


  def zPID_loop(self,z_r,z):
	  # Compute the current error
	  error = z_r - z

	  # UPIDate Differentiator
	  a1 = (2*P.sigma - P.Ts)/(2*P.sigma+P.Ts)
	  a2 = 2/(2*P.sigma+P.Ts)
	  self.differentiator = a1*self.differentiator \
                          + a2*(z -self.z_d1)

	  # Update Integrator
	  if abs(self.differentiator) <0.05:
		  self.integrator += (P.Ts/2.0)*(error+self.error_d1)

	  # UPIDate error_d1
	  self.error_d1 = error
	  self.z_d1 = z

	  # PID Control to calculate T
	  theta_r_unsat = self.kp*error - self.kd*self.differentiator + self.ki*self.integrator

	  theta_r_sat = self.saturate(theta_r_unsat)

	#   if self.ki != 0:
	# 	  self.integrator += P.Ts/self.ki*(theta_r_sat-theta_r_unsat)

	  return theta_r_sat

  def saturate(self,u):
    if abs(u) > self.limit:
      u = self.limit*np.sign(u)
    return u
