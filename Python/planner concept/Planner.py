from tkinter import *

import math
import numpy as np


#robot configuration constants
max_extend = 6
min_extend = 2

#maximum angle of center segment
max_angle = math.pi / 6.0 #30 degrees


#foot locations are represented as (X,Y, theta) tuples theta is relative to the x axis
feet = [(-2,0,0),(0,0,0),(2,0,0)]

#target location for the center foot
target = (5,7,0)

#set up window
root = Tk()   
root.wm_title("Planner Prototype")
root.config(background="#000000")





#start the GUI
root.mainloop()
