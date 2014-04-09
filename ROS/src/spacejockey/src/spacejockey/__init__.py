import roslib
roslib.load_manifest('spacejockey')
import rospy
import rospkg
from urdf_parser_py.urdf import URDF

#ROS Global Config stuff

#get configuration constants from the param server
#this class recursively parses configuration attributes into properties, letting us use the easy . syntax
class ParamNode:
    def __init__(self, **entries): 
        self.__dict__.update(entries)
        for key in self.__dict__:
        	if type(self.__dict__[key]) is dict:
        		self.__dict__[key] = ParamNode(**self.__dict__[key])

def config(prefix = ""):
	return ParamNode(**rospy.get_param(prefix))

try:
	urdf = URDF.from_parameter_server()
except KeyError:
	urdf = URDF.from_xml_file(rospkg.RosPack().get_path('spacejockey') + '/resources/spacejockey.urdf')