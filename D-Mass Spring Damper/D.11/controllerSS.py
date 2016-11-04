import sys
import numpy as np
import param as P

class controllerSS:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the SS_ctrl object
      self.SSCtrl = SS_ctrl(P.K,P.kr,P.z0,P.F_max)
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
  def __init__(self,K,kr,z0,limit):
      self.zdot = 0.0              # Difference term
      self.z_d1 = z0               # Last z term
      self.K = K                   # Closed loop SS gains
      self.kr = kr                 # Input gain
      self.limit = limit           # Maxiumum force


  def SS_loop(self,z_r,z):


      # Update Differentiator
      a1 = (2*P.sigma - P.Ts)/(2*P.sigma+P.Ts)
      a2 = 2/(2*P.sigma+P.Ts)
      self.zdot = a1*self.zdot + a2*(z-self.z_d1)

      self.z_d1 = z

      # Construct the state
      x = np.matrix([[z],
                     [self.zdot]])

      # Compute the state feedback controller
      F_unsat = -self.K*x + self.kr*z_r

      F_sat = self.saturate(F_unsat)
      return F_sat.item(0)

  def saturate(self,u):
    if abs(u) > self.limit:
      u = self.limit*np.sign(u)
    return u
