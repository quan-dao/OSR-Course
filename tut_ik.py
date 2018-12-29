import rospy
import numpy as np
import openravepy as orpy
import tf.transformations as tr


env = orpy.Environment()
env.Load('/home/mquan/ros/src/cri/osr_course_pkgs/osr_openrave/worlds/cubes_task.env.xml')
env.SetDefaultViewer()
robot = env.GetRobot('robot')
manipulator = robot.SetActiveManipulator('gripper')
robot.SetActiveDOFs(manipulator.GetArmIndices())
np.set_printoptions(precision=6, suppress=True)
# Box to ilustrate elbow up/down
with env:
	box = orpy.RaveCreateKinBody(env, '')
	box.SetName('box')
	box.InitFromBoxes(np.array([[0.5, 0.3, 1, 0.01, 0.04, 0.22]]), True)
	env.AddKinBody(box)
	box_centroid = box.ComputeAABB().pos()
	print "box centroid", box_centroid
	
ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=orpy.IkParameterization.Type.Transform6D)
if not ikmodel.load():
	ikmodel.autogenerate()
Tgrasp = tr.quaternion_matrix([ 0.5,  0.5,  0.5, -0.5])
Tgrasp[:3,3] = box_centroid

# Checkout IK solutions
# solutions = manipulator.FindIKSolutions(Tgrasp, 0)
# print "All solutions: "
# print solutions

solutions = manipulator.FindIKSolutions(Tgrasp, orpy.IkFilterOptions.CheckEnvCollisions)
print "Collisions free solutions"
print solutions

qgrasp = solutions[0]
robot.SetActiveDOFValues(qgrasp)
# Close the gripper and grab the box
taskmanip = orpy.interfaces.TaskManipulation(robot)
taskmanip.CloseFingers()
robot.WaitForController(0)
robot.Grab(box)

twist = np.array([0, 0, 0.01, 0, 0, 0])

link_idx = [l.GetName() for l in robot.GetLinks()].index('robotiq_85_base_link')
link_origin = robot.GetLink('robotiq_85_base_link').GetTransform()[:3,3]
J = np.zeros((6,6))
q = robot.GetActiveDOFValues()
print "initial q = ", q
for i in range(10):
	J[:3,:] = robot.ComputeJacobianTranslation(link_idx, link_origin)[:,:6]
	J[3:,:] = robot.ComputeJacobianAxisAngle(link_idx)[:,:6]
	qdot = np.linalg.solve(J, twist)
	q[:6] += qdot
	print "q = ", q
	robot.SetActiveDOFValues(q)
	raw_input('Press Enter for next differential IK step')

raw_input('Press Enter to finish')
