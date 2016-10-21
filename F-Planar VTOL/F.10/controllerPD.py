import sys
import numpy as np
import param as P

class controllerPD:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the PD_ctrl object
      self.zCtrl = zPID_ctrl(P.z_kp,P.z_kd,P.z_ki,P.zv0)
      self.thetaCtrl = thetaPID_ctrl(P.th_kp,P.th_kd,P.th_ki,P.theta0)
      self.hCtrl = hPID_ctrl(P.h_kp,P.h_kd,P.h_ki,P.h0)
      # kp is the proportional gain
      # kd is the derivative gain
      # y0 is the initial position of the state

  def getForces(self,y_r,y):
      # y_r is the referenced input
      # y is the current state
      z_r = y_r[0]
      z = y[0]
      theta = y[2]
      theta_r = self.zCtrl.zPID_loop(z_r,z)
      h_r = y_r[1]
      h = y[1]
      T = self.thetaCtrl.thetaPID_loop(theta_r,theta)
      F = self.hCtrl.hPID_loop(h_r,h,T)
      Fr = F/2.0 + T/2.0/P.de
      Fl = F/2.0 - T/2.0/P.de
      return [Fr,Fl,theta_r]

class thetaPID_ctrl:
  def __init__(self,kp,kd,ki,theta0):
	  self.differentiator = 0.0    # Difference term
	  self.integrator = 0.0
	  self.theta_d1 = theta0       # Delayed y output
	  self.error_d1 = 0.0          # Delayed error
	  self.kp = kp                 # Proportional control gain
	  self.kd = kd                 # Derivative control gain
	  self.ki = ki


  def thetaPID_loop(self,theta_r,theta):
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
	#   if abs(self.differentiator) <0.05:
	# 	  self.integrator += (P.Ts/2.0)*(error+self.error_d1)

	  self.integrator += (P.Ts/2.0)*(error+self.error_d1)

	  # PID Control to calculate T
	  T = self.kp*error - self.kd*self.differentiator + self.ki*self.integrator

	  return T

class zPID_ctrl:
  def __init__(self,kp,kd,ki,z0):
      self.differentiator = 0.0    # Difference term
      self.integrator = 0.0        # Integral term
      self.z_d1 = z0               # Delayed y output
      self.error_d1 = 0.0          # Delayed error
      self.kp = kp                 # Proportional control gain
      self.kd = kd                 # Derivative control gain
      self.ki = ki                 # Integral controal gain


  def zPID_loop(self,z_r,z):
	  # Compute the current error
	  error = z_r - z

	  # UPIDate Differentiator
	  a1 = (2*P.sigma - P.Ts)/(2*P.sigma+P.Ts)
	  a2 = 2/(2*P.sigma+P.Ts)
	  self.differentiator = a1*self.differentiator \
                          + a2*(z -self.z_d1)

	  # Update Integrator
	#   if abs(self.differentiator) <0.05:
	# 	  self.integrator += (P.Ts/2.0)*(error+self.error_d1)

	  self.integrator += (P.Ts/2.0)*(error+self.error_d1)

      # UPIDate error_d1
	  self.error_d1 = error
	  self.z_d1 = z

	  # PID Control to calculate T
	  theta_r_unsat = self.kp*error - self.kd*self.differentiator + self.ki*self.integrator

	  theta_r_sat = self.saturate(theta_r_unsat)

	  if self.ki != 0:
		  self.integrator += P.Ts/self.ki*(theta_r_sat-theta_r_unsat)

	  return theta_r_sat

  def saturate(self,u):
    if abs(u) > P.theta_max:
      u = P.theta_max*np.sign(u)
    return u

class hPID_ctrl:
  def __init__(self,kp,kd,ki,h0):
      self.differentiator = 0.0    # Difference term
      self.integrator = 0.0        # Integral term
      self.h_d1 = h0               # Delayed y output
      self.error_d1 = 0.0          # Delayed error
      self.kp = kp                 # Proportional control gain
      self.kd = kd                 # Derivative control gain
      self.ki = ki                 # Integral controal gain


  def hPID_loop(self,h_r,h,T):
	  # Compute the current error
	  error = h_r - h

	  # UPIDate Differentiator
	  a1 = (2*P.sigma - P.Ts)/(2*P.sigma+P.Ts)
	  a2 = 2/(2*P.sigma+P.Ts)
	  self.differentiator = a1*self.differentiator \
                          + a2*(h -self.h_d1)

	  # Update Integrator
	#   if abs(self.differentiator) <0.05:
	# 	  self.integrator += (P.Ts/2.0)*(error+self.error_d1)

	  self.integrator += (P.Ts/2.0)*(error+self.error_d1)

	  # UPIDate error_d1
	  self.error_d1 = error
	  self.h_d1 = h

	  # PID Control to calculate T
	  F_unsat = self.kp*error - self.kd*self.differentiator + self.ki*self.integrator + P.F0

	  F_sat = self.saturate([F_unsat,T])

	  if self.ki != 0:
		  self.integrator += P.Ts/self.ki*(F_sat-F_unsat)

	  return F_sat

  def saturate(self,u):
      F = u[0]
      T = u[1]
    #   print ("before:",F, T)
      Fr = F/2.0 + T/2.0/P.de
      Fl = F/2.0 - T/2.0/P.de
      if abs(Fl) > P.Fl_max:
          Fl = P.Fl_max*np.sign(u)[0]
          F = T/P.de + 2*Fl
          Fr = F/2.0 + T/2.0/P.de
      if abs(Fr) > P.Fr_max:
          Fr = P.Fr_max*np.sign(u)[0]
          F = 2*Fr - T/P.de
          Fl = F/2.0 - T/2.0/P.de
      F = Fr + Fl
      T = P.de*(Fr - Fl)
    #   print ("after:",F,T)
      return F
