import numpy as np 
import param as P



class robotArmDynamics:

    def __init__(self):

        # Initial state conditions
        self.state = np.matrix([[P.theta0],     # Theta initial orientation
                                [P.thetadot0]]) # Thetadot initial velocity 

    def propagateDynamics(self,u):
        # P.Ts is the time step between function calls.
        # u contains the force and/or torque input(s).

        # RK4 integration
        k1 = self.Derivatives(self.state, u)
        k2 = self.Derivatives(self.state + P.Ts/2*k1, u)
        k3 = self.Derivatives(self.state + P.Ts/2*k2, u)
        k4 = self.Derivatives(self.state + P.Ts*k3, u)
        self.state += P.Ts/6 * (k1 + 2*k2 + 2*k3 + k4)


    # Return the derivatives of the continuous states
    def Derivatives(self,state,u):

        # States and forces
        theta = state.item(0)
        thetadot = state.item(1)
        tau = u[0]
        
        thetaddot = (3/P.m/P.ell**2)*(tau-P.b*thetadot-P.m*P.g*P.ell/2*np.cos(theta))

        xdot = np.matrix([[thetadot],[thetaddot]])

        return xdot


    # Returns the observable states
    def Outputs(self):
        # Return them in a list and not a matrix
        return self.state[0:2].T.tolist()[0]

    # Returns all current states
    def States(self):
        # Return them in a list and not a matrix
        return self.state.T.tolist()[0]