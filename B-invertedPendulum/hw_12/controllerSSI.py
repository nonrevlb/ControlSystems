import sys
import numpy as np 
import param as P

class controllerSSI:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the SS_ctrl object
      self.SSICtrl = SSI_ctrl(P.K,P.ki,P.z0,P.theta0,P.F_max)
      # K is the closed loop SS gains
      # ki is the integrator gain
      # y0 is the initial position of the state

  def getForces(self,y_r,y):
      # y_r is the referenced input
      # y is the current state  
      z_r = y_r[0]
      z = y[0]
      theta = y[1]
      
      F = self.SSICtrl.SSI_loop(z_r,z,theta) # Calculate the force output
      return [F]


class SSI_ctrl:
  def __init__(self,K,ki,z0,theta0,limit):
      self.zdot = 0.0              # Difference term
      self.thetadot = 0.0          # Difference term
      self.integrator = 0.0        # Integrator term
      self.z_d1 = z0               # Last z term
      self.theta_d1 = theta0       # Last theta term
      self.error_d1 = 0.0
      self.K = K                   # Closed loop SS gains
      self.ki = ki                 # Integrator gain
      self.limit = limit           # Maxiumum force
      self.input_disturbance = 0.5 # Input disturbance, N


  def SSI_loop(self,z_r,z,theta):

      error = z_r-z

      # Update Differentiator
      a1 = (2*P.sigma - P.Ts)/(2*P.sigma+P.Ts)
      a2 = 2/(2*P.sigma+P.Ts)
      self.zdot = a1*self.zdot + a2*(z-self.z_d1)

      self.thetadot = a1*self.thetadot + a2*(theta-self.theta_d1)
      self.integrator += (P.Ts/2.0)*(error+self.error_d1)

      # Update delays
      self.z_d1 = z
      self.theta_d1 = theta 
      self.error_d1 = error

      # Construct the state
      x = np.matrix([[z],
                     [theta],
                     [self.zdot],
                     [self.thetadot]])

      # Compute the state feedback controller
      F_unsat = -self.K*x -self.ki*self.integrator + self.input_disturbance

      F_sat = self.saturate(F_unsat)

      if self.ki !=0:
        self.integrator += P.Ts/self.ki*(F_sat-F_unsat)
      return F_sat.item(0)

  def saturate(self,u):
    if abs(u) > self.limit:
      u = self.limit*np.sign(u)
    return u 








