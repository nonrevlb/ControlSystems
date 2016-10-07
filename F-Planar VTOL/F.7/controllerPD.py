import sys
import numpy as np
import param as P

class controllerPD:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the PD_ctrl object
      self.hCtrl = hPD_ctrl(P.kp,P.kd,P.h0,P.F_max)
      # kp is the proportional gain
      # kd is the derivative gain
      # y0 is the initial position of the state

  def getForces(self,y_r,y):
      # y_r is the referenced input
      # y is the current state
      h_r = y_r[0]
      h = y[1]
      hdot = y[4]
      F = self.hCtrl.hPD_loop(h_r,h,hdot) # Calculate the force output
      return [F]

class hPD_ctrl:
  def __init__(self,kp,kd,h0,limit):
      self.differentiator = 0.0    # Difference term
      self.h_d1 = h0               # Delayed y output
      self.error_d1 = 0.0          # Delayed error
      self.kp = kp                 # Proportional control gain
      self.kd = kd                 # Derivative control gain
      self.limit = limit           # Maxiumum theta


  def hPD_loop(self,h_r,h,hdot):
      # Compute the current error
      error = h_r - h

      # Update error_d1
      self.error_d1 = error

      # PD Control to calculate T
      F_r_unsat = self.kp*error - self.kd*hdot

      F_r_sat = self.saturate(F_r_unsat)

      return F_r_sat

  def saturate(self,u):
    if abs(u) > self.limit:
      u = self.limit*np.sign(u)
    return u
