import sys
import numpy as np 
import param as P

class controllerPD:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the PD_ctrl object
      self.thetaCtrl=thetaPD_ctrl(P.kp,P.kd,P.theta0,P.error_max) 
      # kp is the proportional gain
      # kd is the derivative gain
      # y0 is the initial position of the state

  def getForces(self,y_r,y):
      # y_r is the referenced input
      # y is the current state  
      theta_r = y_r[0]
      theta = y[0]
      tau = self.thetaCtrl.thetaPD_loop(theta_r,theta) # Calculate the force output
      return [tau]


class thetaPD_ctrl:
  def __init__(self,kp,kd,theta0,limit):
      self.thetadot = 0.0    # Difference term
      self.theta_d1 = theta0       # Delayed y output
      self.error_d1 = 0.0          # Delayed error
      self.kp = kp                 # Proportional control gain
      self.kd = kd                 # Derivative control gain
      self.limit = limit           # Max torque

  def thetaPD_loop(self,theta_r,theta):
      # Compute the current error
      error = theta_r - theta  

      # Update thetadot
      a1 = (2*P.sigma - P.Ts)/(2*P.sigma+P.Ts)
      a2 = 2/(2*P.sigma+P.Ts)
      self.thetadot = a1*self.thetadot \
                          + a2*(error -self.error_d1)

      # Update error_d1
      self.error_d1 = error 

      # The controllers designed on feedback linearization 
      # with have an equilibrium force since
      # tau_tilde = tau - tau_e
      # tau = tau_tilde + tau_e
      tau_e = P.m*P.g*P.ell*np.cos(theta)/2

      # PD Control to calculate T
      tau_unsat = self.kp*error + self.kd*self.thetadot + tau_e

      tau_sat = self.saturate(tau_unsat)
      return tau_sat


  def saturate(self,u):
    if abs(u) > self.limit:
      u = self.limit*np.sign(u)
    return u



