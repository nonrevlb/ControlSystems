import time
import sys
import numpy as np
import matplotlib.pyplot as plt
import param as P
from slider_input import Sliders

# The Animation.py file is kept in the parent directory,
# so the parent directory path needs to be added.
sys.path.append('..')
from dynamics import PendulumDynamics
from animation import PendulumAnimation

t_start = 0.0   # Start time of simulation
t_end = 50.0    # End time of simulation
t_Ts = P.Ts     # Simulation time step
t_elapse = 0.1  # Simulation time elapsed between each iteration
t_pause = 0.01  # Pause between each iteration

user_input = Sliders()              # Instantiate Sliders class
simAnimation = PendulumAnimation()  # Instantiate Animate class
dynam = PendulumDynamics()          # Instantiate Dynamics class

t = t_start               # Declare time variable to keep track of simulation time elapsed

while t < t_end:

	plt.ion()                           # Make plots interactive
	plt.figure(user_input.fig.number)   # Switch current figure to user_input figure
	plt.pause(0.001)                    # Pause the simulation to detect user input

	# The dynamics of the model will be propagated in time by t_elapse
	# at intervals of t_Ts.
	t_temp = t +t_elapse
	while t < t_temp:
		dynam.propagateDynamics(        # Propagate the dynamics of the model in time
			user_input.getInputValues())
		t += t_Ts                       # Update time elapsed

	plt.figure(simAnimation.fig.number) # Switch current figure to animation figure
	simAnimation.drawPendulum(          # Update animation with current user input
		dynam.Outputs())


	# time.sleep(t_pause)
