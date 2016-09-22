import numpy as np 
import param as P



class PendulumDynamics:

    def __init__(self):

        # Initial state conditions
        self.state = np.matrix([[P.z0],         # z initial position
                                [P.theta0],     # Theta initial orientation
                                [P.zdot0],      # zdot initial velocity
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
        z = state.item(0)
        theta = state.item(1)
        zdot = state.item(2)
        thetadot = state.item(3)
        F = u[0]

        # ctheta and stheta are used multiple times. They are
        # precomputed and stored in another variable to increase
        # efficiency. 
        ctheta = np.cos(theta);
        stheta = np.sin(theta);

        # The equations of motion.
        M = np.matrix([[P.m1+P.m2,         P.m1*P.ell*ctheta],
                       [P.m1*P.ell*ctheta, P.m1*P.ell**2    ]])

        C = np.matrix([[P.m1*P.ell*thetadot**2*stheta + F-P.b*zdot],
                       [P.m1*P.g*P.ell*stheta                     ]])

        tmp = np.linalg.inv(M)*C 


        zddot = tmp.item(0)
        thetaddot = tmp.item(1)

        xdot = np.matrix([[zdot],[thetadot],[zddot],[thetaddot]])

        return xdot


    # Returns the observable states
    def Outputs(self):
        # Return them in a list and not a matrix
        return self.state[0:2].T.tolist()[0]

    # Returns all current states
    def States(self):
        # Return them in a list and not a matrix
        return self.state.T.tolist()[0]