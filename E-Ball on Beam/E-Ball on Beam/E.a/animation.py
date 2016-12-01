import matplotlib.pyplot as plt
import sys
import matplotlib.patches as mpatches
import numpy as np
sys.path.append('hw_a/')
import param as P

class Animation:

	def __init__(self):
		self.flagInit = True                  # Used to indicate initialization
		self.fig, self.ax = plt.subplots()    # Initializes a figure and axes object
		self.handle = []                      # Initializes a list object that will
		                                      # be used to contain handles to the
		                                      # patches and line objects.
		plt.axis([-(P.l+P.r)+P.l/2,P.l+P.r+P.l/2, -(P.l+P.r), P.l+P.r]) # Change the x,y axis limits

		# Draw pendulum is the main function that will call the functions:
		# drawCart, drawCircle, and drawRod to create the animation.
	def drawSystem(self, u):
		# Process inputs to function
		z= u[0]        # Horizontal position of cart, m
		theta = u[1]   # Angle of pendulum, rads

		self.drawBeam(theta)
		self.drawBall(z,theta)
		#self.ax.axis('equal') # This will cause the image to not distort

		# After each function has been called, initialization is over.
		if self.flagInit == True:
			self.flagInit = False

	def drawBeam(self,theta):
		X = [0,P.l*np.cos(theta)]                  # X data points
		Y = [0, P.l*np.sin(theta)] # Y data points

		# When the class is initialized, a line object will be
		# created and added to the axes. After initialization, the
		# line object will only be updated.
		if self.flagInit == True:
			# Create the line object and append its handle
			# to the handle list.
			line, =self.ax.plot(X,Y,lw = 2, c = 'red')
			self.handle.append(line)
		else:
			self.handle[0].set_xdata(X)               # Update the line
			self.handle[0].set_ydata(Y)

	def drawBall(self,z,theta):
		x = z*np.cos(theta)-P.r*np.sin(theta)        # x coordinate
		y = z*np.sin(theta)+P.r*np.cos(theta)   # y coordinate
		xy = (x,y)                                   # Center of circle

		# When the class is initialized, a CirclePolygon patch object will
		# be created and added to the axes. After initialization, the
		# CirclePolygon patch object will only be updated.
		if self.flagInit == True:
			# Create the CirclePolygon patch and append its handle
			# to the handle list
			self.handle.append(mpatches.CirclePolygon(xy,
				radius = P.r, resolution = 30,
				fc = 'blue', ec = 'black'))
			self.ax.add_patch(self.handle[1])  # Add the patch to the axes
		else:
			self.handle[1]._xy=xy



# Used see the animation.
if __name__ == "__main__":

    simAnimation = Animate()              # Create Animate object
    z = 0.0                               # Position of cart, m
    theta = 0.0*180/np.pi                 # Angle of pendulum, rads
    simAnimation.drawSystem([z,theta])  # Draw the pendulum
    plt.show()
