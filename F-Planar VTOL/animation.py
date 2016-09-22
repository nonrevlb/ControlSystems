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
		plt.axis([-(P.r+P.d),11*P.d+P.r, -(P.d+P.r), 11*P.d+P.r]) # Change the x,y axis limits
		plt.plot([-(P.r+P.d),11*P.d+P.r],[0,0],'k')

		# Draw pendulum is the main function that will call the functions:
		# drawCart, drawCircle, and drawRod to create the animation.
	def drawSystem(self, u):
		# Process inputs to function
		zv = u[0]        # Horizontal position of cart, m
		h = u[1]
		theta = u[2]   # Angle of pendulum, rads
		zt = u[0]

		self.drawTarget(zt)
		self.drawArms(zv,h,theta)
		self.drawPod(zv,h,theta)
		self.drawRightRotor(zv,h,theta)
		self.drawLeftRotor(zv,h,theta)
		#self.ax.axis('equal') # This will cause the image to not distort

		# After each function has been called, initialization is over.
		if self.flagInit == True:
			self.flagInit = False

	def drawTarget(self,zt):
		x = zt-P.wt/2.0  # x coordinate
		y = 0      # y coordinate
		xy = (x,y)     # Bottom left corner of rectangle

		# When the class is initialized, a Rectangle patch object will be
		# created and added to the axes. After initialization, the Rectangle
		# patch object will only be updated.
		if self.flagInit == True:
			# Create the Rectangle patch and append its handle
			# to the handle list
			self.handle.append(mpatches.Rectangle(xy,
				P.wt,P.ht, fc = 'red', ec = 'black'))
			self.ax.add_patch(self.handle[0]) # Add the patch to the axes
		else:
			self.handle[0].set_xy(xy)         # Update patch

	def drawArms(self,zv,h,theta):
		X = [zv-P.d*np.cos(theta),zv+P.d*np.cos(theta)]                  # X data points
		Y = [h-P.d*np.sin(theta),h+P.d*np.sin(theta)] # Y data points

		# When the class is initialized, a line object will be
		# created and added to the axes. After initialization, the
		# line object will only be updated.
		if self.flagInit == True:
			# Create the line object and append its handle
			# to the handle list.
			line, =self.ax.plot(X,Y,lw = 2, c = 'blue')
			self.handle.append(line)
		else:
			self.handle[1].set_xdata(X)               # Update the line
			self.handle[1].set_ydata(Y)

	def drawPod(self,zv,h,theta):
		x = zv-(P.wc/2)*np.cos(theta)+(P.hc/2)*np.sin(theta)  # x coordinate
		y = h-(P.wc/2)*np.sin(theta)-(P.hc/2)*np.cos(theta)      # y coordinate
		xy = (x,y)     # Bottom left corner of rectangle

		# When the class is initialized, a Rectangle patch object will be
		# created and added to the axes. After initialization, the Rectangle
		# patch object will only be updated.
		if self.flagInit == True:
			# Create the Rectangle patch and append its handle
			# to the handle list
			self.handle.append(mpatches.Rectangle(xy,
				P.wc,P.hc,angle=theta*180/np.pi, fc = 'blue', ec = 'black'))
			self.ax.add_patch(self.handle[2]) # Add the patch to the axes
		else:
			self.handle[2].set_xy(xy)         # Update patch
			self.handle[2]._angle = theta*180/np.pi

	def drawRightRotor(self,zv,h,theta):
		x = zv+P.d*np.cos(theta)       # x coordinate
		y = h+P.d*np.sin(theta)   # y coordinate
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
			self.ax.add_patch(self.handle[3])  # Add the patch to the axes
		else:
			self.handle[3]._xy=xy

	def drawLeftRotor(self,zv,h,theta):
		x = zv-P.d*np.cos(theta)       # x coordinate
		y = h-P.d*np.sin(theta)   # y coordinate
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
			self.ax.add_patch(self.handle[4])  # Add the patch to the axes
		else:
			self.handle[4]._xy=xy



# Used see the animation.
if __name__ == "__main__":
    simAnimation = Animate()
    simAnimation.drawSystem([P.zv0,P.h0,P.theta0,P.zt0])  # Draw the pendulum
    plt.show()
