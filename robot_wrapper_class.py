import numpy as np
import openravepy as orpy


class RobotWrapper(object):
	"""docstring for RobotWrapper"""
	def __init__(self, robot_name, _env, end_effector_name='gripper'):
		self.env = _env
		# initialize robot
		self.robot = self.env.GetRobot(robot_name)
		self.manipulator = self.robot.SetActiveManipulator(end_effector_name)
		self.robot.SetActiveDOFs(self.manipulator.GetArmIndices())
		
		# initialize IK model
		self.ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(self.robot, 
						iktype=orpy.IkParameterization.Type.Transform6D)
		if not self.ikmodel.load():
			print "[INFO] generating IK model"
			self.ikmodel.autogenerate()

		# create path planner
		self.planner = orpy.RaveCreatePlanner(self.env, 'birrt') # Using bidirectional RRT
		self.params = orpy.Planner.PlannerParameters()
		self.params.SetRobotActiveJoints(self.robot)

		# get robot controller
		self.controller = self.robot.GetController()

		# create interface for manipulation task
		self.taskmanip = orpy.interfaces.TaskManipulation(self.robot)

	def moveIt(self, qtarget):
		self.params.SetGoalConfig(qtarget)
		self.params.SetPostProcessing('ParabolicSmoother', '<_nmaxiterations>40</_nmaxiterations>')
		self.planner.InitPlan(self.robot, self.params)
		# Plan a trajectory
		traj = orpy.RaveCreateTrajectory(self.env, '')
		self.planner.PlanPath(traj)
		# Execute the trajectory
		self.controller.SetPath(traj)
		self.robot.WaitForController(0)

	def pickUp(self, box_kinbody):
		self.taskmanip.CloseFingers()
		self.robot.WaitForController(0)
		self.robot.Grab(box_kinbody)
		self.robot.WaitForController(0)

	def putDown(self, box_kinbody):
		self.taskmanip.ReleaseFingers()
		self.robot.WaitForController(0)
		# self.robot.ReleaseAllGrabbed()
		self.robot.Release(box_kinbody)
		self.robot.WaitForController(0)

	def solveIK(self, Ttarget):
		'''
		Solve IK to get 1 solution that is closest to the current joints position & collision free
		'''
		solutions = self.manipulator.FindIKSolutions(Ttarget, orpy.IkFilterOptions.CheckEnvCollisions)
		# solutions = self.manipulator.FindIKSolutions(Ttarget, 0)
		# print "[DEBUG] num solutions: ", solutions.shape
		# choose closest solution
		q_current = self.robot.GetActiveDOFValues()
		min_dist = 1e5
		i_qtarget = -1
		for i, cand in enumerate(solutions):
			dist = np.sum(np.abs(cand - q_current))
			if dist < min_dist:
				min_dist = dist
				i_qtarget = i
		return solutions[i_qtarget]

	def move2Ttarget(self, Ttarget):
		qtarget = self.solveIK(Ttarget)
		self.moveIt(qtarget)

	def testTtarget(self, Ttarget):
		solutions = self.manipulator.FindIKSolutions(Ttarget, 0)
		print "[DEBUG] num solutions: ", solutions.shape
		self.robot.SetActiveDOFValues(solutions[0])


def box2Ttarget(box_kinbody, grab=True, height_offset=0.):
	box_centroid = box_kinbody.ComputeAABB().pos()
	print "grasp box at ", box_centroid
	# Create grasp pose
	Ttarget = box_kinbody.GetTransform()
	# # rotate x & z around y by pi
	Ttarget[:3, 0] *= -1.
	# Ttarget[:3, 1] *= -1.  
	Ttarget[:3, 2] *= -1.
	Ttarget[:3, 3] = box_centroid
	Ttarget[2, 3] += 0.01
	if not grab:
		assert height_offset > 0.
		Ttarget[2, 3] += height_offset
	return Ttarget

'''
if __name__ == "__main__":
	env = orpy.Environment()
	env.Load('/home/mquan/ros/src/cri/osr_course_pkgs/osr_openrave/worlds/cubes_task.env.xml')
	env.SetDefaultViewer()

	robot = RobotWrapper('robot', env)  # initialize robot
	
	boxes = [env.GetKinBody("cube0%d"%i) for i in range(1, 4)]
	h = 0.051
	delta_h = h 
	for i in range(2):
		# generate target
		Ttarget_grab = box2Ttarget(boxes[i])
		Ttarget_release = box2Ttarget(boxes[2], False, h)
		# move to box i
		robot.move2Ttarget(Ttarget_grab)
		# pick up box i
		robot.pickUp(boxes[i])
		# release at box 2
		robot.move2Ttarget(Ttarget_release)
		robot.putDown()
		# increase h
		h += delta_h


	raw_input("Press Enter to finish")
'''
