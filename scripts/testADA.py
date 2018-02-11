#!/usr/bin/env python
import adapy
import openravepy
import orcdchomp.orcdchomp
import rospy
import numpy
import IPython

def main():
	rospy.init_node('planning', anonymous = True)
	env, robot = adapy.initialize(
			sim = True,
			attach_viewer = 'rviz'
			)
	m_chomp = openravepy.RaveCreateModule(env, 'orcdchomp')
	
	table = env.ReadKinBodyXMLFile('models/furniture/rolly-table.iv')
	env.Add(table)
	table.SetTransform([0.70711,0.70711,0,0,0,0,0])

	mug = env.ReadKinBodyXMLFile('models/objects/mug3.iv')
	env.Add(mug)
	mug.SetTransform([1,0,0,0,0,0,0.7])
	T_robot_w = robot.GetTransform()
	T_robot_w[0][3] -= 0.3
	T_robot_w[1][3] += 0.4
	T_robot_w[2][3] += 0.71
	robot.SetTransform(T_robot_w)
	
	T_mug_world = mug.GetTransform()

	T_palm_grip = numpy.array(
	   [[ 1., 0., 0., 0. ],
	    [ 0., 1., 0., 0.  ],
	    [ 0., 0., 1., 0.125 ],
	    [ 0., 0., 0., 1. ]])
	
	T_mug_palm  = numpy.array(
	   [[ 0, -1,  0, 0.000 ],
	    [ 0,  0, -1, 0.075 ],
	    [ 1,  0,  0, 0.100 ],
	    [ 0,  0,  0, 1     ]])

	T_grip_world = numpy.dot(T_mug_world, 
		numpy.linalg.inv(T_mug_palm), 
		numpy.linalg.inv(T_palm_grip))
	try:
		ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot,
		   iktype=openravepy.IkParameterization.Type.Transform6D)
		q_goal = robot.arm.FindIKSolution(T_grip_world, 0)
		print "q_goal:", q_goal
		if not ikmodel.load():
		   print "I don't have an ikmodel"
		   ikmodel.autogenerate()
	except: 
		print "I am not able to plan to goal"


	if not m_chomp:
	   raise RuntimeError('no chomp module found!')
	orcdchomp.orcdchomp.bind(m_chomp)
	m_chomp.computedistancefield(kinbody=robot,cache_filename='sdf_tablemug.dat')
	
	try:
	   t = m_chomp.runchomp(robot=robot.arm, n_iter=100, lambda_=100.0, obs_factor=500.0,
	      adofgoal=q_goal, no_collision_exception=True)
	except RuntimeError as ex:
	   print ex
	   t = None

	while not rospy.is_shutdown():
		rospy.loginfo("I am in planning")
		rospy.sleep(2)

if __name__ == "__main__":
	main()
			
