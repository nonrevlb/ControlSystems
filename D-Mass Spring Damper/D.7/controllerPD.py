import sys
import numpy as np
import param as P

class controllerPD:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the PD_ctrl object
      self.zCtrl = zPD_ctrl(P.kp,P.kd,P.z0,P.F_max)
      # kp is the proportional gain
      # kd is the derivative gain
      # y0 is the initial position of the state

  def getForces(self,y_r,y):
      # y_r is the referenced input
      # y is the current state
      z_r = y_r[0]
      z = y[0]
      v = y[1]
      F = self.zCtrl.zPD_loop(z_r,z,v) # Calculate the force output
      return [F]

class zPD_ctrl:
  def __init__(self,kp,kd,z0,limit):
      self.differentiator = 0.0    # Difference term
      self.z_d1 = z0               # Delayed y output
      self.error_d1 = 0.0          # Delayed error
      self.kp = kp                 # Proportional control gain
      self.kd = kd                 # Derivative control gain
      self.limit = limit           # Maxiumum theta


  def zPD_loop(self,z_r,z,v):
      # Compute the current error
      error = z_r - z

      # Update Differentiator
      a1 = (2*P.sigma - P.Ts)/(2*P.sigma+P.Ts)
      a2 = 2/(2*P.sigma+P.Ts)
      self.differentiator = a1*self.differentiator \
                          + a2*(error -self.error_d1)

      # Update error_d1
      self.error_d1 = error

      # PD Control to calculate T
      F_r_unsat = self.kp*error - self.kd*v

      F_r_sat = self.saturate(F_r_unsat)

      return F_r_sat

  def saturate(self,u):
    if abs(u) > self.limit:
      u = self.limit*np.sign(u)
    return u
