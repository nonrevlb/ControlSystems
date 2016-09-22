import matplotlib.pyplot as plt
import sys 
import matplotlib.patches as mpatches
import numpy as np 
sys.path.append('hw_a/')
import param as P

class robotArmAnimation:

	def __init__(self):
		self.flagInit = True                  # Used to indicate initialization
		self.fig, self.ax = plt.subplots()    # Initializes a figure and axes object
		self.handle = []                      # Initializes a list object that will 
		                                      # be used to contain handles to the  
		                                      # patches and line objects.
		axis_scale = 1.5
		plt.axis([-axis_scale*P.ell,axis_scale*P.ell,  # Change the x,y axis limits
				 -(axis_scale-1)*P.ell, axis_scale*P.ell])
		plt.plot([-P.ell,P.ell],[0,0],'k:')

		# Draws robotArm 
	def drawRobotArm(self, u):
		# Process inputs to function
		theta= u[0]    # Angle of robot arm, m

		X = [0, P.ell*np.cos(theta)] # X data points                
		Y = [0, P.ell*np.sin(theta)] # Y data points

		# When the class is initialized, a ine object will be
		# created and added to the axes. After initialization, the 
		# line object will only be updated.
		if self.flagInit == True:  
			# Create the line object and append its handle
			# to the handle list.                  
			line, =self.ax.plot(X,Y,lw = 3, c = 'skyblue')
			self.handle.append(line)
			self.flagInit = False
		else:
			self.handle[0].set_xdata(X)               # Update the line
			self.handle[0].set_ydata(Y)
		




# Used see the animation.
if __name__ == "__main__":

    simAnimation = robotArmAnimation()  # Create Animate object
    theta = 45.0*np.pi/180.0              # Angle of robot arm, rads
    simAnimation.drawRobotArm([theta])  # Draw the Planar VTOL  
    plt.show()