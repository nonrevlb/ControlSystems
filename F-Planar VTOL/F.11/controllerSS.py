import sys
import numpy as np
import param as P

class controllerSS:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the SS_ctrl object
      self.SSCtrl = SS_ctrl(P.K_lon,P.kr_lon,P.K_lat,P.kr_lat,P.zv0,P.theta0,P.h0,P.F_max)
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


class SS_ctrl:
  def __init__(self,K_lon,kr_lon,K_lat,kr_lat,z0,theta0,h0,limit):
      self.zdot = 0.0              # Difference term
      self.thetadot = 0.0          # Difference term
      self.hdot =0.0
      self.z_d1 = z0               # Last z term
      self.theta_d1 = theta0       # Last theta term
      self.h_d1 = h0
      self.kr_lon = kr_lon
      self.K_lon = K_lon                   # Closed loop SS gains
      self.kr_lat = kr_lat                 # Input gain
      self.K_lat = K_lat
      self.limit = limit


  def SS_loop(self,z_r,z,theta,h_r,h):


      # Update Differentiator
      a1 = (2*P.sigma - P.Ts)/(2*P.sigma+P.Ts)
      a2 = 2/(2*P.sigma+P.Ts)
      self.zdot = a1*self.zdot + a2*(z-self.z_d1)

      self.thetadot = a1*self.thetadot + a2*(theta-self.theta_d1)

      self.hdot = a1*self.hdot + a2*(h-self.h_d1)

      self.z_d1 = z
      self.theta_d1 = theta
      self.h_d1 = h

      # Construct the state
      x_lat = np.matrix([[z],
                     [theta],
                     [self.zdot],
                     [self.thetadot]])

      x_lon = np.matrix([[h],
                        [self.hdot]])

      # Compute the state feedback controller
      F = P.F0 - self.K_lon*(x_lon - P.x0_lon) + self.kr_lon*(h_r - P.C_lon*P.x0_lon)
      T = P.T0 - self.K_lat*(x_lat - P.x0_lat) + self.kr_lat*(z_r - P.C_lat*P.x0_lat)
      Fr = F/2.0 + T/2.0/P.de
      Fl = F/2.0 - T/2.0/P.de
      Fr_sat = self.saturate(Fr)
      Fl_sat = self.saturate(Fl)
      return [Fr_sat.item(0),Fl_sat.item(0)]

  def saturate(self,u):
    if u > self.limit:
      u = self.limit*np.sign(u)
    if u < 0:
      u = 0.0*np.sign(u)
    return u
