import sys
import numpy as np
import param as P

class controllerSS:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the SS_ctrl object
      self.SSCtrl = SS_ctrl(P.K_lon,P.ki_lon,P.L_lon,P.K_lat,P.ki_lat,P.L_lat,P.zv0,P.theta0,P.h0,P.F_max)
      # K is the closed loop SS gains
      # kr is the input gain
      # y0 is the initial position of the state

  def getForces(self,y_r,y):
      # y_r is the referenced input
      # y is the current state
      z_r = y_r[0]
      z = y[0]
      theta = y[2]
      h_r = y_r[1]
      h = y[1]

      u = self.SSCtrl.SS_loop(z_r,z,theta,h_r,h) # Calculate the force output
      return u

  def getObsStates(self):
    return self.SSCtrl.getObsStates()


class SS_ctrl:
  def __init__(self,K_lon,ki_lon,L_lon,K_lat,ki_lat,L_lat,z0,theta0,h0,limit):
      self.xhat_lon = np.matrix([[P.h0],  # h
                             [0.0]]) # h dot
      self.xhat_lat = np.matrix([[P.zv0],  # z
                             [P.theta0],  # theta
                             [0.0],  # zdot
                             [0.0]]) # theta dot
      self.F_d1 = 0.0
      self.T_d1 = 0.0
      self.integrator_z = 0.0
      self.integrator_h = 0.0
      self.error_z_d1 = 0.0
      self.error_h_d1 = 0.0
      self.ki_lon = ki_lon
      self.K_lon = K_lon                   # Closed loop SS gains
      self.L_lon = L_lon
      self.ki_lat = ki_lat                 # Input gain
      self.K_lat = K_lat
      self.L_lat = L_lat
      self.limit = limit


  def SS_loop(self,z_r,z,theta,h_r,h):

      # Lon Observer
      N = 10
      for i in range(N):
        self.xhat_lon += P.Ts/N*(P.A_lon*(self.xhat_lon-P.x0_lon) + \
          P.B_lon*(self.F_d1-P.F0)+\
          self.L_lon*((np.matrix([[h]]) - P.C_lon*self.xhat_lon)))

      # Lat Observer
      for i in range(N):
        self.xhat_lat += P.Ts/N*(P.A_lat*(self.xhat_lat-P.x0_lat) + \
          P.B_lat*(self.T_d1-P.T0)+\
          self.L_lat*((np.matrix([[z],[theta]]) - P.C_lat*self.xhat_lat)))

      error_z = z_r-self.xhat_lat.item(0)
      error_h = h_r-self.xhat_lon.item(0)

      self.integrator_z += (P.Ts/2.0)*(error_z+self.error_z_d1)
      self.integrator_h += (P.Ts/2.0)*(error_h+self.error_h_d1)

      self.error_z_d1 = error_z
      self.error_h_d1 = error_h

      # Compute the state feedback controller
      F = P.F0 - self.K_lon*(self.xhat_lon - P.x0_lon) - self.ki_lon*self.integrator_h
      T = P.T0 - self.K_lat*(self.xhat_lat - P.x0_lat) - self.ki_lat*self.integrator_z

      Fr = F/2.0 + T/2.0/P.de
      Fl = F/2.0 - T/2.0/P.de
      Fr_sat = self.saturate(Fr)
      Fl_sat = self.saturate(Fl)

      F_sat = Fr + Fl
      T_sat = P.de*(Fr-Fl)

      self.F_d1 = F_sat
      self.T_d1 = T_sat

      if self.ki_lon !=0:
        self.integrator_h += P.Ts/self.ki_lon*(F_sat-F)
      if self.ki_lat !=0:
        self.integrator_z += P.Ts/self.ki_lat*(T_sat-T)

      return [Fr_sat.item(0),Fl_sat.item(0)]

  def getObsStates(self):
    return [self.xhat_lat.tolist(),self.xhat_lon.tolist()]

  def saturate(self,u):
    if u > self.limit:
      u = self.limit*np.sign(u)
    if u < 0:
      u = 0.0*np.sign(u)
    return u
