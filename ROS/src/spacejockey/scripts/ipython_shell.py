#!/usr/bin/env python  
import rospy
import IPython
rospy.init_node('ipython_shell')
IPython.embed()