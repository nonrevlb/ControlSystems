import numpy as np
import param as P



class Dynamics:

    def __init__(self):

        # Initial state conditions
        self.state = np.matrix([[P.zv0],         # z initial position
                                [P.h0],         # h initial position
                                [P.theta0],     # Theta initial orientation
                                [P.zvdot0],      # zdot initial velocity
                                [P.hdot0],      # h intitial velocity
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
        h = state.item(1)
        theta = state.item(2)
        zdot = state.item(3)
        hdot = state.item(4)
        thetadot = state.item(5)
        Fr = u[0]
        Fl = u[1]

        # ctheta and stheta are used multiple times. They are
        # precomputed and stored in another variable to increase
        # efficiency.
        ctheta = np.cos(theta);
        stheta = np.sin(theta);

        # The equations of motion.
        M = np.matrix([[P.mc+2*P.mr, 0, 0],
                       [0, P.mc+2*P.mr, 0],
                       [0, 0, P.Jc+2*P.mr*P.d*P.d]])

        C = np.matrix([[-(Fr+Fl)*stheta-P.mu*zdot],
                       [(Fr+Fl)*ctheta-(P.mc+2*P.mr)*P.g],
                       [P.d*(Fr-Fl)]])

        tmp = np.linalg.inv(M)*C


        zddot = tmp.item(0)
        hddot = tmp.item(1)
        thetaddot = tmp.item(2)

        xdot = np.matrix([[zdot],[hdot],[thetadot],[zddot],[hddot],[thetaddot]])

        return xdot


    # Returns the observable states
    def Outputs(self):
        # Return them in a list and not a matrix
        return self.state[0:6].T.tolist()[0]

    # Returns all current states
    def States(self):
        # Return them in a list and not a matrix
        return self.state.T.tolist()[0]
