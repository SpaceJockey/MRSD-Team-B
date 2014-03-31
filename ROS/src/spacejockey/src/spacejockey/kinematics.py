
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



"""
void TreeFkSolverPosFull_recursive::addFrameToMap(const map<string, double>& joint_pos, 
						  map<string, tf::Stamped<Frame> >& tf_out,
						  const tf::Stamped<KDL::Frame>& previous_frame,
						  const SegmentMap::const_iterator this_segment,
						  bool flatten_tree)
{
  // get pose of this segment
  tf::Stamped<KDL::Frame> this_frame;
  double jnt_p = 0;
  if (this_segment->second.segment.getJoint().getType() != Joint::None){
    map<string, double>::const_iterator jnt_pos = joint_pos.find(this_segment->second.segment.getJoint().getName());
    if (jnt_pos == joint_pos.end()){
      ROS_DEBUG("Warning: TreeFKSolverPosFull Could not find value for joint '%s'. Skipping this tree branch", this_segment->first.c_str());
      return;
    }
    jnt_p = jnt_pos->second;
  }
  this_frame = tf::Stamped<KDL::Frame>(previous_frame * this_segment->second.segment.pose(jnt_p), ros::Time(), previous_frame.frame_id_);

  if (this_segment->first != tree.getRootSegment()->first)
    tf_out.insert(make_pair(this_segment->first, tf::Stamped<KDL::Frame>(this_frame, ros::Time(), previous_frame.frame_id_)));

  // get poses of child segments
  for (vector<SegmentMap::const_iterator>::const_iterator child=this_segment->second.children.begin(); child !=this_segment->second.children.end(); child++){
    if (flatten_tree)
      addFrameToMap(joint_pos, tf_out, this_frame, *child, flatten_tree);
    else
      addFrameToMap(joint_pos, tf_out, tf::Stamped<KDL::Frame>(KDL::Frame::Identity(), ros::Time(), this_segment->first), *child, flatten_tree);
  }      
}
"""
#TODO: Parse URDF for joint infos....
#from urdf_parser_py.urdf import URDF
def FK(joint_pos, checklimits=True):
	
	return ((f_trans, f_rot), (c_trans, c_rot), (b_trans, b_rot))

def IK((f_trans, f_rot), (c_trans, c_rot), (b_trans, b_rot), checklimits=True):

	return

"""return the normalized torques on each joint given a joint_position, and unit vector in the center joint frame"""
def unit_jacobian(joint_pos, vector):
	return