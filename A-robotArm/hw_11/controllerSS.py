import sys
import numpy as np 
import param as P

class controllerSS:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the SS_ctrl object
      self.thetaCtrl=thetaSS_ctrl(P.K,P.kr,P.theta0,P.tau_max) 
      # K is the closed loop SS gains
      # kr is the input gain
      # y0 is the initial position of the state

  def getForces(self,y_r,y):
      # y_r is the referenced input
      # y is the current state  
      theta_r = y_r[0]
      theta = y[0]
      tau = self.thetaCtrl.thetaSS_loop(theta_r,theta) # Calculate the force output
      return [tau]


class thetaSS_ctrl:
  def __init__(self,K,kr,theta0,limit):
      self.thetadot = 0.0          # Difference term
      self.error_d1 = 0.0          # Delayed error
      self.limit = limit           # Max torque
      self.K = K                   # SS gains
      self.kr = kr                 # Input gain

  def thetaSS_loop(self,theta_r,theta):
      # Compute the current error
      error = theta_r - theta  

      # Update Differentiator
      a1 = (2*P.sigma - P.Ts)/(2*P.sigma+P.Ts)
      a2 = 2/(2*P.sigma+P.Ts)
      self.thetadot = a1*self.thetadot \
                          + a2*(error -self.error_d1)

      # Update error_d1
      self.error_d1 = error 

      # Construct the state
      x = np.matrix([[theta],
                     [-self.thetadot]])

      # Compute the equilibrium torque tau_e
      tau_e = P.m*P.g*P.ell*np.cos(theta)/2

      # Compute the state feedback controller
      tau_tilde = -self.K*x+self.kr*theta_r

      # Compute total torque
      tau = self.saturate(tau_e+tau_tilde)

      return tau.item(0) 

 

  def saturate(self,u):
    if abs(u) > self.limit:
      u = self.limit*np.sign(u)
    return u



