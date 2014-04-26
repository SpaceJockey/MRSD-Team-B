import rospy
import rospkg
from urdf_parser_py.urdf import URDF
from tf_cache import LocalTfCache
import std_redirect

#ROS Global Config stuff

#get configuration constants from the param server
#this class recursively parses configuration attributes into properties, letting us use the easy . syntax
class _ParamNode:
    def __init__(self, **entries): 
        self.__dict__.update(entries)
        for key in self.__dict__:
        	if type(self.__dict__[key]) is dict:
        		self.__dict__[key] = _ParamNode(**self.__dict__[key])

def config(prefix = ""):
	return _ParamNode(**rospy.get_param(prefix))

import geometry

with std_redirect.SysRedirect(stderr=std_redirect.devnull): #squelch annoying URDF warnings
	try:
		urdf = URDF.from_parameter_server()
	except KeyError:
		urdf = URDF.from_xml_file(rospkg.RosPack().get_path('spacejockey') + '/resources/spacejockey.urdf')

import kinematics