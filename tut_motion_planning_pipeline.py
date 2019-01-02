import numpy as np
import openravepy as orpy
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure


env = orpy.Environment()
env.Load('/home/mquan/ros/src/cri/osr_course_pkgs/osr_openrave/worlds/cubes_task.env.xml')
env.SetDefaultViewer()

# Robot stuff ==================================================================== #
robot = env.GetRobot('robot')
manipulator = robot.SetActiveManipulator('gripper')
robot.SetActiveDOFs(manipulator.GetArmIndices())

# initialize IK model
ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=orpy.IkParameterization.Type.Transform6D)
if not ikmodel.load():
	ikmodel.autogenerate()

# Create the planner
planner = orpy.RaveCreatePlanner(env, 'birrt') # Using bidirectional RRT
params = orpy.Planner.PlannerParameters()
params.SetRobotActiveJoints(robot)

controller = robot.GetController()

# create an interface for manipulation task
taskmanip = orpy.interfaces.TaskManipulation(robot)
# ======================================================================= #

# Box pose
boxes = [env.GetKinBody("cube0%d"%i) for i in range(1, 4)]


def robotMoveIt(qtarget):
	params.SetGoalConfig(qtarget)
	params.SetPostProcessing('ParabolicSmoother', '<_nmaxiterations>40</_nmaxiterations>')
	planner.InitPlan(robot, params)

	# Plan a trajectory
	traj = orpy.RaveCreateTrajectory(env, '')
	planner.PlanPath(traj)

	# Execute the trajectory
	# controller = robot.GetController()
	controller.SetPath(traj)
	robot.WaitForController(0)


def robotGrab(box_kinbody):
	# Close the gripper and grab the box
	# taskmanip = orpy.interfaces.TaskManipulation(robot)
	taskmanip.CloseFingers()
	robot.WaitForController(0)
	robot.Grab(box_kinbody)


def robotRelease(box_kinbody):
	taskmanip.ReleaseFingers()
	robot.WaitForController(0)
	robot.ReleaseAllGrabbed()
	# robot.WaitForController(0)


def robotIK(Ttarget):
	# solve IK for qgrasp
	solutions = manipulator.FindIKSolutions(Ttarget, orpy.IkFilterOptions.CheckEnvCollisions)
	# solutions = manipulator.FindIKSolutions(Ttarget, 0)
	print "num solutions: ", solutions.shape
	# choose closest solution
	q_current = robot.GetActiveDOFValues()
	min_dist = 1e5
	i_qtarget = -1
	for i, cand in enumerate(solutions):
		dist = np.sum(np.abs(cand - q_current))
		if dist < min_dist:
			min_dist = dist
			i_qtarget = i
	return solutions[i_qtarget]


def robotMove2Box(box_kinbody, grab=True, height_offset=0):
	box_centroid = box_kinbody.ComputeAABB().pos()
	print "grasp box at ", box_centroid
	# Create grasp pose
	Ttarget = box_kinbody.GetTransform()
	# rotate x & z around y by pi
	Ttarget[:3, 0] *= -1.  
	Ttarget[:3, 2] *= -1.
	Ttarget[:3, 3] = box_centroid
	if not grab:
		assert height_offset > 0.
		Ttarget[2, 3] += height_offset
	# solve IK for qgrasp
	qtarget = robotIK(Ttarget)
	# move robot to qgrasp & grasp the box
	robotMoveIt(qtarget)
	if grab:
		robotGrab(box_kinbody)
	else:
		robotRelease(box_kinbody)




'''
# Plot joint value
times = np.arange(0, traj.GetDuration(), 0.01)
qvect = np.zeros((len(times), robot.GetActiveDOF()))
spec = traj.GetConfigurationSpecification()
for i in range(len(times)):
	trajdata = traj.Sample(times[i])
	qvect[i,:] = spec.ExtractJointValues(trajdata, robot, manipulator.GetArmIndices(), 0)

fig = Figure()
canvas = FigureCanvas(fig)
ax = fig.add_subplot(1,1,1)
ax.plot(times, qvect)
ax.grid(True)
ax.set_xlabel('Time [s]')
ax.set_ylabel('Joint Values [rad]')
canvas.print_figure('joint_values.png')
'''

robotMove2Box(boxes[1])
raw_input("Press Enter to release at box2")
robotMove2Box(boxes[2], False, 0.051)
raw_input("Press Enter to pick up next box")
# move robot to neutral place
robotMoveIt(np.zeros(6))
raw_input("wait")
robotMove2Box(boxes[0])
raw_input("Press Enter to release at box2")
robotMove2Box(boxes[2], False, 0.102)

raw_input("Press Enter to finish")
