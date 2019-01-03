import numpy as np
import openravepy as orpy


np.random.seed(4)

# Environment stuff
env = orpy.Environment() # create the environment
env.Load('/home/mquan/ros/src/cri/osr_course_pkgs/osr_openrave/worlds/pick_and_place.env.xml')
env.SetViewer('qtcoin')
# orpy.RaveSetDebugLevel(orpy.DebugLevel.Debug) # set output level to debug

def create_box(T, color = [0, 0.6, 0]):
  box = orpy.RaveCreateKinBody(env, '')
  box.SetName('box')
  box.InitFromBoxes(np.array([[0,0,0,0.035,0.03,0.005]]), True)
  g = box.GetLinks()[0].GetGeometries()[0]
  g.SetAmbientColor(color)
  g.SetDiffuseColor(color)
  box.SetTransform(T)
  env.Add(box,True)
  return box


T = np.eye(4)
container_center = np.array([0.4, 0.2, 0.195])
# Destination
T[:3, 3] = container_center + np.array([0, -0.5, 0])
destination0 = create_box(T, color = [0, 0, 0.6])
T[:3, 3] = container_center + np.array([0, -0.6, 0])
destination1 = create_box(T, color = [0, 0, 0.6])

# Generate random box positions
boxes = []
nbox_per_layer = 2
n_layer = 20
h = container_center[2]
for i in range(n_layer):
    nbox_current_layer = 0
    while nbox_current_layer < nbox_per_layer:
        theta = np.random.rand()*np.pi
        T[0, 0] = np.cos(theta)
        T[0, 1] = -np.sin(theta)
        T[1, 0] = np.sin(theta)
        T[1, 1] = np.cos(theta)
        T[0, 3] = container_center[0] + (np.random.rand()-0.5)*0.2
        T[1, 3] = container_center[1] + (np.random.rand()-0.5)*0.1
        T[2, 3] = h
        box = create_box(T)
        if env.CheckCollision(box):
            env.Remove(box)
        else:
            boxes.append(box)
            nbox_current_layer += 1
    h += 0.011

boxes.sort(key=lambda box:box.GetTransform()[2, 3], reverse=True)
# =========================================================================== #

# Robot stuff
robot = env.GetRobots()[0]
manipprob = orpy.interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs

manip = robot.SetActiveManipulator('gripper') # set the manipulator to leftarm
robot.SetActiveDOFs(manip.GetArmIndices())
ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=orpy.IkParameterization.Type.Transform6D)
if not ikmodel.load():
    ikmodel.autogenerate()

# grab 1st box
box = boxes[0]


def boxDisplay(box):
	color = [1, 0, 0.5]
	g = box.GetLinks()[0].GetGeometries()[0]
	g.SetAmbientColor(color)
	g.SetDiffuseColor(color)

boxDisplay(box)


# create Tgrab
def rotByPi(T, axis_idx):
	rot_T = np.zeros(T.shape)
	# rotate Tgrab around its own axis informed by axis_idx (0 = x, 1 = y, 2 = z)
	for i in range(3):
		if i != axis_idx:
			rot_T[:3, i] = -T[:3, i]
		else:
			rot_T[:3, i] = T[:3, i]
	return rot_T

def rotX(T, theta):
	rot = np.eye(4)
	rot[1, 1] = np.cos(theta)
	rot[1, 2] = -np.sin(theta)
	rot[2, 1] = np.sin(theta)
	rot[2, 2] = np.cos(theta)
	return np.dot(T, rot)


def rotY(T, theta):
	rot = np.eye(4)
	rot[0, 0] = np.cos(theta)
	rot[0, 2] = np.sin(theta)
	rot[2, 0] = -np.sin(theta)
	rot[2, 2] = np.cos(theta)
	return np.dot(T, rot)


def box2Target(box, axis_idx=1):
	T = rotByPi(box.GetTransform(), axis_idx)  # rotate around y
	T[:3, 3] = box.ComputeAABB().pos()
	T[2, 3] += 0.005
	T[-1, -1] = 1
	return T


def dest2Target(dest, h_offset, axis_idx=1):
	assert h_offset > 0
	T = rotByPi(dest.GetTransform(), axis_idx)  # rotate around y
	T[:3, 3] = dest.ComputeAABB().pos()
	T[2, 3] += h_offset
	T[-1, -1] = 1
	return T

Tpick = box2Target(box)
print "Tpick = \n", Tpick

Tplace = dest2Target(destination1, 0.011)
print "Tplace = \n", Tplace

Tee = manip.GetEndEffectorTransform()
print "Tee = \n", Tee

raw_input("Press Enter to start")
sol = manip.FindIKSolution(Tpick, orpy.IkFilterOptions.CheckEnvCollisions) # get collision-free solution
# sol = manip.FindIKSolutions(Tgrab, 0)

print "[Tgrab] IK solution: ", sol


sol2 = manip.FindIKSolution(Tplace, orpy.IkFilterOptions.CheckEnvCollisions) # get collision-free solution
print "[Tdrop] IK solution: ", sol2


manipprob.MoveManipulator(goal=sol) # call motion planner with goal joint angles
robot.WaitForController(0) # wait
robot.Grab(box)
orpy.raveLogInfo('Move to place location')
manipprob.MoveManipulator(goal=sol2) # call motion planner with goal joint angles
robot.WaitForController(0) # wait
robot.Release(box)

# ============================== for 2nd box
raw_input("Press Enter to continue with 2nd box")
# env.Remove(box)
box = boxes[1]
boxDisplay(box)
Tpick = box2Target(box)
print "Tpick = \n", Tpick

# Tplace = dest2Target(destination1, 0.011*2)
Tplace = dest2Target(destination0, 0.011)
print "Tplace = \n", Tplace

Tee = manip.GetEndEffectorTransform()
print "Tee = \n", Tee

it = 0
sol = None
while it < 20:
	sol = manip.FindIKSolution(Tpick, orpy.IkFilterOptions.CheckEnvCollisions) # get collision-free solution
	print "it = %d, Tpick: \n" % it, Tpick
	if sol is not None:
		print "Bingoooooooo"
		break
	# tilt gripper
	Tpick = rotY(Tpick, 5. * np.pi / 180.)
	it += 1

print "[Tgrab] IK solution: ", sol


sol2 = manip.FindIKSolution(Tplace, orpy.IkFilterOptions.CheckEnvCollisions) # get collision-free solution
print "[Tdrop] IK solution: ", sol2

manipprob.MoveManipulator(goal=sol) # call motion planner with goal joint angles
robot.WaitForController(0) # wait
robot.Grab(box)
orpy.raveLogInfo('Move to place location')
manipprob.MoveManipulator(goal=sol2) # call motion planner with goal joint angles
robot.WaitForController(0) # wait
robot.Release(box)


raw_input("Press Enter to finish.")
