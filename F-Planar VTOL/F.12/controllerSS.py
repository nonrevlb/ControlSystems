import sys
import numpy as np
import param as P

class controllerSS:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the SS_ctrl object
      self.SSCtrl = SS_ctrl(P.K_lon,P.ki_lon,P.K_lat,P.ki_lat,P.zv0,P.theta0,P.h0,P.F_max)
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
  def __init__(self,K_lon,ki_lon,K_lat,ki_lat,z0,theta0,h0,limit):
      self.zdot = 0.0              # Difference term
      self.thetadot = 0.0          # Difference term
      self.hdot =0.0
      self.integrator_z = 0.0
      self.integrator_h = 0.0
      self.z_d1 = z0               # Last z term
      self.theta_d1 = theta0       # Last theta term
      self.h_d1 = h0
      self.error_z_d1 = 0.0
      self.error_h_d1 = 0.0
      self.ki_lon = ki_lon
      self.K_lon = K_lon                   # Closed loop SS gains
      self.ki_lat = ki_lat                 # Input gain
      self.K_lat = K_lat
      self.limit = limit


  def SS_loop(self,z_r,z,theta,h_r,h):

      error_z = z_r-z
      error_h = h_r-h

      # Update Differentiator
      a1 = (2*P.sigma - P.Ts)/(2*P.sigma+P.Ts)
      a2 = 2/(2*P.sigma+P.Ts)
      self.zdot = a1*self.zdot + a2*(z-self.z_d1)

      self.thetadot = a1*self.thetadot + a2*(theta-self.theta_d1)

      self.hdot = a1*self.hdot + a2*(h-self.h_d1)

      self.integrator_z += (P.Ts/2.0)*(error_z+self.error_z_d1)
      self.integrator_h += (P.Ts/2.0)*(error_h+self.error_h_d1)

      self.z_d1 = z
      self.theta_d1 = theta
      self.h_d1 = h
      self.error_z_d1 = error_z
      self.error_h_d1 = error_h

      # Construct the state
      x_lat = np.matrix([[z],
                     [theta],
                     [self.zdot],
                     [self.thetadot]])

      x_lon = np.matrix([[h],
                        [self.hdot]])

      # Compute the state feedback controller
      F = P.F0 - self.K_lon*(x_lon - P.x0_lon) - self.ki_lon*self.integrator_h
      T = P.T0 - self.K_lat*(x_lat - P.x0_lat) - self.ki_lat*self.integrator_z

      Fr = F/2.0 + T/2.0/P.de
      Fl = F/2.0 - T/2.0/P.de
      Fr_sat = self.saturate(Fr)
      Fl_sat = self.saturate(Fl)

      F_sat = Fr + Fl
      T_sat = P.de*(Fr-Fl)

      if self.ki_lon !=0:
        self.integrator_h += P.Ts/self.ki_lon*(F_sat-F)
      if self.ki_lat !=0:
        self.integrator_z += P.Ts/self.ki_lat*(T_sat-T)

      return [Fr_sat.item(0),Fl_sat.item(0)]

  def saturate(self,u):
    if u > self.limit:
      u = self.limit*np.sign(u)
    if u < 0:
      u = 0.0*np.sign(u)
    return u
