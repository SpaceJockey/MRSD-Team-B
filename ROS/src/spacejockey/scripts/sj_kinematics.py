
# These are all stubs, TODO: fill them in

#Todo: replace tf::Transform[] with tf::Pose?

#return the joint angles necessary to reach the desired location
#to get a desired camera pose, call with [foot0_tf, foot1_tf, footPoseFromCamera(camera_tf)], [1,1,0]
def InverseKinematics(tf::Transform[] foot_locs, bool[] foot_status_desired):
	return joint_angles
	
#return a foot transform necessary to reach a desired camera pose
def footPoseFromCamera(tf cameraPose):
	return foot_tf
	
#Do Jacobian analysis of gravity vector
#gravity vector must be in the center segment frame
def GetTorques(float[] joint_angles, tf::Vector3 gravity_vector):
	return torque_vals