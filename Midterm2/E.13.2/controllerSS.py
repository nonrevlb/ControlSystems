import sys
import numpy as np
import param as P

class controllerSS:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the SS_ctrl object
      self.SSCtrl = SS_ctrl(P.K,P.kr,P.Lo,P.z0,P.theta0,P.F_max)
      # K is the closed loop SS gains
      # kr is the input gain
      # y0 is the initial position of the state

  def getForces(self,y_r,y):
      # y_r is the referenced input
      # y is the current state
      z_r = y_r[0]
      z = y[2]
      theta = y[3]

      F = self.SSCtrl.SS_loop(z_r,z,theta) # Calculate the force output
      return [F]

  def getObsStates(self):
    return self.SSCtrl.getObsStates()


class SS_ctrl:
  def __init__(self,K,kr,L,z0,theta0,limit):
      self.xhat = np.matrix([[0.0],  # z
                             [0.0],  # theta
                             [0.0],  # zdot
                             [0.0]]) # theta dot
      self.F_d1 = 0.0
      self.K = K                   # Closed loop SS gains
      self.kr = kr                 # Input gain
      self.L = L
      self.limit = limit           # Maxiumum force


  def SS_loop(self,z_r,z,theta):

      # Observer
      N = 10
      for i in range(N):
        self.xhat += P.Ts/N*(P.A*(self.xhat-P.x0) + \
          P.B*(self.F_d1-P.F0)+\
          self.L*((np.matrix([[z],[theta]]) - P.C*self.xhat)))

      # Compute the state feedback controller
      F_unsat =P.F0 -self.K*(self.xhat-P.x0) + self.kr*(z_r-P.Cr*P.x0)

      F_sat = self.saturate(F_unsat)

      self.F_d1 = F_sat

      return F_sat.item(0)

  def getObsStates(self):
    return self.xhat.tolist()

  def saturate(self,u):
    if abs(u) > self.limit:
      u = self.limit*np.sign(u)
    return u
