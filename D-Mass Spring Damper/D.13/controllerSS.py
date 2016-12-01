import sys
import numpy as np
import param as P

class controllerSS:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the SS_ctrl object
      self.SSCtrl = SS_ctrl(P.K,P.ki,P.L,P.z0,P.F_max)
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

  def getObsStates(self):
      return self.SSCtrl.getObsStates()


class SS_ctrl:
  def __init__(self,K,ki,L,z0,limit):
      self.xhat = np.matrix([[0.0],     # z
                             [0.0]])    # zdot
      self.F_d1 = 0.0
      self.integrator = 0.0
      self.error_d1 = 0.0
      self.K = K                   # Closed loop SS gains
      self.ki = ki                 # Input gain
      self.L = L
      self.limit = limit           # Maxiumum force


  def SS_loop(self,z_r,z):

      # Observer
      N = 10
      for i in range(N):
        self.xhat += P.Ts/N*(P.A*self.xhat + \
          P.B*(self.F_d1)+\
          self.L*(np.matrix([[z]]) - P.C*self.xhat))

      error = z_r-self.xhat.item(0)

      self.integrator += (P.Ts/2.0)*(error+self.error_d1)

      self.error_d1 = error

      # Compute the state feedback controller
      F_unsat = -self.K*self.xhat - self.ki*self.integrator

      F_sat = self.saturate(F_unsat)

      self.F_d1 = F_sat

      if self.ki !=0:
          self.integrator += P.Ts/self.ki*(F_sat-F_unsat)

      return F_sat.item(0)

  def getObsStates(self):
    return self.xhat.tolist()

  def saturate(self,u):
    if abs(u) > self.limit:
      u = self.limit*np.sign(u)
    return u
