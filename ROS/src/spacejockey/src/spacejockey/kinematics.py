
import roslib
roslib.load_manifest('spacejockey')
import rospy
import tf

#pull in the link specs from our URDF file
from urdf_parser_py.urdf import URDF
robot = URDF.from_parameter_server()

def printRobot():
	print(robot)
	return

def FK(joint_pos, checklimits=True):
	
	return ((f_trans, f_rot), (c_trans, c_rot), (b_trans, b_rot))

def IK((f_trans, f_rot), (c_trans, c_rot), (b_trans, b_rot), checklimits=True):

	return

"""return the normalized torques on each joint given a joint_position, and unit vector in the center joint frame"""
def unit_jacobian(joint_pos, vector):
	return