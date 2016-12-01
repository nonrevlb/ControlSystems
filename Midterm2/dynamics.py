import numpy as np
import param as P

class Dynamics:

    def __init__(self):

        # Initial state conditions
        self.state = np.matrix([[P.zdot0],         # z initial position
                                [P.thetadot0],     # Theta initial orientation
                                [P.z0],      # zdot initial velocity
                                [P.theta0]]) # Thetadot initial velocity

    def propagateDynamics(self,u):
        # P.Ts is the time step between function calls.

        # RK4 integration
        k1 = self.Derivatives(self.state, u)
        k2 = self.Derivatives(self.state + P.Ts/2.0*k1, u)
        k3 = self.Derivatives(self.state + P.Ts/2.0*k2, u)
        k4 = self.Derivatives(self.state + P.Ts*k3, u)
        self.state += P.Ts/6.0 * (k1 + 2.0*k2 + 2.0*k3 + k4)


    # Return the derivatives of the continuous states
    def Derivatives(self,state,u):

        # States and forces
        zdot = state.item(0)
        thdot = state.item(1)
        z = state.item(2)
        th = state.item(3)

        F = u[0]

        M = np.matrix([[(P.mc+P.m), P.m*P.L*np.cos(th)],
                        [np.cos(th), P.L ]])

        RHS = np.matrix([[-P.b*zdot + P.m*P.L*thdot**2*np.sin(th) + F],
                        [-P.g*np.sin(th)]])

        ddot = np.linalg.inv(M)*RHS
        zddot = ddot.item(0)
        thddot = ddot.item(1)

        xdot = np.matrix([[zddot],[thddot],[zdot],[thdot]])

        return xdot


    # Returns the observable states
    def Outputs(self):
        # Return them in a list and not a matrix
        return self.state[0:4].T.tolist()[0]

    # Returns all current states
    def States(self):
        # Return them in a list and not a matrix
        return self.state.T.tolist()[0]
