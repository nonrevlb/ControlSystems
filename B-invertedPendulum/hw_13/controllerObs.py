import sys
import numpy as np 
import param as P

class controllerObs:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the SS_ctrl object
      self.ObsCtrl = Obs_ctrl(P.K,P.ki,P.L,P.z0,P.theta0,P.F_max)
      # K is the closed loop SS gains
      # ki is the integrator gain
      # L is the observer gain
      # y0 is the initial position of the state

  def getForces(self,y_r,y):
      # y_r is the referenced input
      # y is the current state  
      z_r = y_r[0]
      z = y[0]
      theta = y[1]
      
      F = self.ObsCtrl.Obs_loop(z_r,z,theta) # Calculate the force output
      return [F]

  def getObsStates(self):
    return self.ObsCtrl.getObsStates()


class Obs_ctrl:
  def __init__(self,K,ki,L,z0,theta0,limit):
      self.xhat = np.matrix([[0.0],  # z 
                             [0.0],  # theta
                             [0.0],  # zdot
                             [0.0]]) # theta dot
      self.F_d1 = 0.0                # Delayed input 
      self.integrator=0.0            # Integrator term
      self.error_d1 = 0.0            # Delayed error
      self.limit = limit             # Max torque
      self.K = K                     # SS gains
      self.ki=ki                     # Integrator gain
      self.L = L                     # Obs Gain


  def Obs_loop(self,z_r,z,theta):

      # Observer
      N = 10
      for i in range(N):
        self.xhat += P.Ts/N*(P.A*self.xhat + \
          P.B*(self.F_d1)+\
          self.L*(np.matrix([[z],[theta]]) - P.C*self.xhat))      

      error = z_r-self.xhat.item(0)

      self.integrator += (P.Ts/2.0)*(error+self.error_d1)

      # Update delays
      self.error_d1 = error


      # Compute the state feedback controller
      F_unsat = -self.K*self.xhat -self.ki*self.integrator 

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








