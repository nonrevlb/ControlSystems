import sys
import numpy as np
import param as P

class controllerPD:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the PD_ctrl object
      self.zCtrl = zPD_ctrl(P.z_kp,P.z_kd,P.z0,P.theta_max)
      self.thetaCtrl=thetaPD_ctrl(P.th_kp,P.th_kd,P.theta0,P.F_max)
      # kp is the proportional gain
      # kd is the derivative gain
      # y0 is the initial position of the state

  def getForces(self,y_r,y):
      # y_r is the referenced input
      # y is the current state
      z_r = y_r[0]
      z = y[0]
      zdot = y[2]
      theta = y[1]
      thetadot = y[3]
      theta_r = self.zCtrl.zPD_loop(z_r,z,zdot)
      Fe = P.m1*P.g/P.l*z+P.m2*P.g/2
      F = Fe + self.thetaCtrl.thetaPD_loop(theta_r,theta,thetadot) # Calculate the force output
      return [F, theta_r]


class thetaPD_ctrl:
  def __init__(self,kp,kd,theta0,limit):
      self.differentiator = 0.0    # Difference term
      self.theta_d1 = theta0       # Delayed y output
      self.error_d1 = 0.0          # Delayed error
      self.kp = kp                 # Proportional control gain
      self.kd = kd                 # Derivative control gain
      self.limit = limit           # Maxiumum force


  def thetaPD_loop(self,theta_r,theta,thetadot):
      # Compute the current error
      error = theta_r - theta

      # Update error_d1
      self.error_d1 = error

      # PD Control to calculate T
      F_unsat = self.kp*error - self.kd*thetadot

      F_sat = self.saturate(F_unsat)
      return F_sat

  def saturate(self,u):
    if abs(u) > self.limit:
      u = self.limit*np.sign(u)
    return u

class zPD_ctrl:
  def __init__(self,kp,kd,z0,limit):
      self.differentiator = 0.0    # Difference term
      self.z_d1 = z0               # Delayed y output
      self.error_d1 = 0.0          # Delayed error
      self.kp = kp                 # Proportional control gain
      self.kd = kd                 # Derivative control gain
      self.limit = limit           # Maxiumum theta


  def zPD_loop(self,z_r,z,zdot):
      # Compute the current error
      error = z_r - z

      # Update error_d1
      self.error_d1 = error

      # PD Control to calculate T
      theta_r_unsat = self.kp*error - self.kd*zdot

      return theta_r_unsat

  def saturate(self,u):
    if abs(u) > self.limit:
      u = self.limit*np.sign(u)
    return u
