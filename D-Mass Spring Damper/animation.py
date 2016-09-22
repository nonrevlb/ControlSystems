import matplotlib.pyplot as plt
import sys
import matplotlib.patches as mpatches
import numpy as np
import param as P

class Animation:

	def __init__(self):
		self.flagInit = True                  # Used to indicate initialization
		self.fig, self.ax = plt.subplots()    # Initializes a figure and axes object
		self.handle = []                      # Initializes a list object that will
		                                      # be used to contain handles to the
		                                      # patches and line objects.
		plt.axis([-(P.l+P.w/2+.5),P.l+P.w/2+.5, -.5, P.h+2]) # Change the x,y axis limits
		plt.plot([-(P.l+P.w/2+.5),P.l+P.w/2+.5],[0,0],'k')    # Draw a base line
		plt.plot([-(P.l+P.w/2),-(P.l+P.w/2)],[0, P.h+2],'k')	#Draw Wall

		# Draw pendulum is the main function that will call the functions:
		# drawCart, drawCircle, and drawRod to create the animation.
	def drawSystem(self, u):
		# Process inputs to function
		z= u[0]        # Horizontal position of cart, m

		self.drawBlock(z)
		self.drawSpring(z)
		self.drawDamper(z)
		#self.ax.axis('equal') # This will cause the image to not distort

		# After each function has been called, initialization is over.
		if self.flagInit == True:
			self.flagInit = False


	def drawBlock(self,z):
		x = z-P.w/2.0  # x coordinate
		y = 0      # y coordinate
		xy = (x,y)     # Bottom left corner of rectangle

		# When the class is initialized, a Rectangle patch object will be
		# created and added to the axes. After initialization, the Rectangle
		# patch object will only be updated.
		if self.flagInit == True:
			# Create the Rectangle patch and append its handle
			# to the handle list
			self.handle.append(mpatches.Rectangle(xy,
				P.w,P.h, fc = 'blue', ec = 'darkblue'))
			self.ax.add_patch(self.handle[0]) # Add the patch to the axes
		else:
			self.handle[0].set_xy(xy)         # Update patch

	def drawSpring(self,z):
		X = [-(P.l+P.w/2),z-P.w/2.0]                  # X data points
		Y = [2*P.h/3.0, 2*P.h/3.0] # Y data points

		# When the class is initialized, a line object will be
		# created and added to the axes. After initialization, the
		# line object will only be updated.
		if self.flagInit == True:
			# Create the line object and append its handle
			# to the handle list.
			line, =self.ax.plot(X,Y,lw = 2, c = 'limegreen')
			self.handle.append(line)
		else:
			self.handle[1].set_xdata(X)               # Update the line
			self.handle[1].set_ydata(Y)

	def drawDamper(self,z):
		X = [-(P.l+P.w/2),z-P.w/2.0]                  # X data points
		Y = [P.h/3.0, P.h/3.0] # Y data points

		# When the class is initialized, a line object will be
		# created and added to the axes. After initialization, the
		# line object will only be updated.
		if self.flagInit == True:
			# Create the line object and append its handle
			# to the handle list.
			line, =self.ax.plot(X,Y,lw = 2, c = 'red')
			self.handle.append(line)
		else:
			self.handle[2].set_xdata(X)               # Update the line
			self.handle[2].set_ydata(Y)


# Used see the animation.
if __name__ == "__main__":

    simAnimation = Animate()              # Create Animate object
    z = 0.0                               # Position of blcok, m
    simAnimation.drawPendulum([z])  # Draw the pendulum
    plt.show()
