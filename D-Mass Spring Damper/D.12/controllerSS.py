import sys
import numpy as np
import param as P

class controllerSS:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the SS_ctrl object
      self.SSCtrl = SS_ctrl(P.K,P.ki,P.z0,P.F_max)
      # K is the closed loop SS gains
      # kr is the input gain
      # y0 is the initial position of the state

  def getForces(self,y_r,y):
      # y_r is the referenced input
      # y is the current state
      z_r = y_r[0]
      z = y[0]

      F = self.SSCtrl.SS_loop(z_r,z) # Calculate the force output
      return [F]


class SS_ctrl:
  def __init__(self,K,ki,z0,limit):
      self.zdot = 0.0              # Difference term
      self.integrator = 0.0
      self.z_d1 = z0               # Last z term
      self.error_d1 = 0.0
      self.K = K                   # Closed loop SS gains
      self.ki = ki                 # Input gain
      self.limit = limit           # Maxiumum force
      self.input_disturbance = 0.25


  def SS_loop(self,z_r,z):

      error = z_r-z

      # Update Differentiator
      a1 = (2*P.sigma - P.Ts)/(2*P.sigma+P.Ts)
      a2 = 2/(2*P.sigma+P.Ts)
      self.zdot = a1*self.zdot + a2*(z-self.z_d1)

      self.integrator += (P.Ts/2.0)*(error+self.error_d1)

      self.z_d1 = z
      self.error_d1 = error

      # Construct the state
      x = np.matrix([[z],
                     [self.zdot]])

      # Compute the state feedback controller
      F_unsat = -self.K*x - self.ki*self.integrator + self.input_disturbance

      F_sat = self.saturate(F_unsat)

      if self.ki !=0:
          self.integrator += P.Ts/self.ki*(F_sat-F_unsat)

      return F_sat.item(0)

  def saturate(self,u):
    if abs(u) > self.limit:
      u = self.limit*np.sign(u)
    return u
