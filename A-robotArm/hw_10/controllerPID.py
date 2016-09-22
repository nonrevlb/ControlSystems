import sys
import numpy as np 
import param as P

class controllerPID:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the PD_ctrl object
      self.thetaCtrl=thetaPID_ctrl(P.kp,P.kd,P.ki,P.theta0,P.tau_max) 
      # kp is the proportional gain
      # kd is the derivative gain
      # ki is the integral gain
      # y0 is the initial position of the state
      # P.error_max is the maximum error before saturation

  def getForces(self,y_r,y):
      # y_r is the referenced input
      # y is the current state  
      theta_r = y_r[0]
      theta = y[0]
      tau = self.thetaCtrl.thetaPID_loop(theta_r,theta) # Calculate the force output
      return [tau]


class thetaPID_ctrl:
  def __init__(self,kp,kd,ki,theta0,limit):
      self.thetadot = 0.0    # Difference term
      self.integrator = 0.0        # Integrator term
      self.theta_d1 = theta0       # Delayed y output
      self.error_d1 = 0.0          # Delayed error
      self.kp = kp                 # Proportional control gain
      self.kd = kd                 # Derivative control gain
      self.ki = ki                 # Integral control gain
      self.limit = limit           # Max torque

  def thetaPID_loop(self,theta_r,theta):
      # Compute the current error
      error = theta_r - theta  

      # Update thetadot
      a1 = (2*P.sigma - P.Ts)/(2*P.sigma+P.Ts)
      a2 = 2/(2*P.sigma+P.Ts)
      self.thetadot = a1*self.thetadot \
                          + a2*(error -self.error_d1)

      # Update Integrator
      if abs(self.thetadot) <0.01:
        self.integrator += (P.Ts/2.0)*(error+self.error_d1)

      # Update error_d1
      self.error_d1 = error 

      # The controllers designed on feedback linearization 
      # with have an equilibrium force since
      # tau_tilde = T - tau_e
      # T = tau_tilde + tau_e
      tau_e = P.m*P.g*P.ell*np.cos(theta)/2

      # PD Control to calculate T
      tau_unsat = self.kp*error + self.kd*self.thetadot + \
                self.ki*self.integrator + tau_e

      tau_sat = self.saturate(tau_unsat)

      if self.ki != 0:
        self.integrator += P.Ts/self.ki*(tau_sat-tau_unsat)

      return tau_sat


  def saturate(self,u):
    if abs(u) > self.limit:
      u = self.limit*np.sign(u)
    return u



