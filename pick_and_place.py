import numpy as np
import openravepy as orpy
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure


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

# using link statistics
lmodel = orpy.databases.linkstatistics.LinkStatisticsModel(robot)
if not lmodel.load():
    lmodel.autogenerate()
lmodel.setRobotResolutions(0.01) # set resolution given smallest object is 0.01m
lmodel.setRobotWeights() # set the weights for planning


def boxDisplay(box):
    color = [1, 0, 0.5]
    g = box.GetLinks()[0].GetGeometries()[0]
    g.SetAmbientColor(color)
    g.SetDiffuseColor(color)


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


def transl(T, d):
    _trans = np.eye(4)
    _trans[:3, 3] = d
    return np.dot(T, _trans)


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


# Initialize figure to plot tilt angle
fig = Figure()
canvas = FigureCanvas(fig)
ax = fig.add_subplot(1,1,1)
ax.set_ylim((0, 182))

# grasp frist 5 boxes
flag_dest1 = True
h_offset = 0.011
dest1_boxes_cnt = 1
dest0_boxes_cnt = 1
n_boxes = 20
for i in range(n_boxes):
    box = boxes[i]
    box_centroid = box.ComputeAABB().pos()
    # Compare height of box i & i + 1, if equal, pick the closer one
    next_box = boxes[i + 1]
    next_box_centroid = next_box.ComputeAABB().pos()
    if box_centroid[2] == next_box_centroid[2]:
        if box_centroid[0] > next_box_centroid[0] :
            print "[box %d] has same height as box %d, but further. Swap two boxes" % (i, i + 1)
            # this box is further than the next box, swap two box
            boxes[i + 1] = box
            box = next_box

    boxDisplay(box)
    # create pick and place pose
    Tpick = box2Target(box)
    print "[box %d] Tpick = \n" % i, Tpick

    if flag_dest1:
        Tplace = dest2Target(destination1, h_offset * dest1_boxes_cnt)
        dest1_boxes_cnt += 1  # increase number of boxes in destination1
    else:
        Tplace = dest2Target(destination0, h_offset * dest0_boxes_cnt)
        dest0_boxes_cnt += 1
    print "[box %d] Tplace = \n" % i, Tplace

    # Find q_pick
    q_pick = manip.FindIKSolution(Tpick, orpy.IkFilterOptions.CheckEnvCollisions)
    if q_pick is None:
        it = 0
        Tpick = rotY(Tpick, 5. * np.pi / 180.)
        while it < 20:
            q_pick = manip.FindIKSolution(Tpick, orpy.IkFilterOptions.CheckEnvCollisions) # get collision-free solution
            print "[box %d] [q_pick] it = %d, Tpick: \n" % (i, it), Tpick
            if q_pick is not None:
                print "Bingo. Tilting gripper success!!!"
                break
            # tilt gripper
            Tpick = rotY(Tpick, 5. * np.pi / 180.)
            it += 1
    # still no solution for q_pick, change axis for generating original Tpick
    if q_pick is None:
        it = 0
        Tpick = box2Target(box, 0)  # rot box frame around x (instead of y)
        while it < 20:
            q_pick = manip.FindIKSolution(Tpick, orpy.IkFilterOptions.CheckEnvCollisions) # get collision-free solution
            print "[box %d] [q_pick] it = %d, Tpick: \n" % (i, it), Tpick
            if q_pick is not None:
                print "Bingo. Changing axis & tilting gripper success!!!"
                break
            # tilt gripper
            Tpick = rotY(Tpick, 5. * np.pi / 180.)
            it += 1
    # still no solution, change pick up point
    if q_pick is None:
        it = 0
        Tpick = box2Target(box)  # reset Tpick
        while it < 3:
            q_pick = manip.FindIKSolution(Tpick, orpy.IkFilterOptions.CheckEnvCollisions) # get collision-free solution
            print "it = %d, Tpick: \n" % it, Tpick
            if q_pick is not None:
                print "Bingo. Slide pick up point success!!!"
                break
            # slide the grip
            Tpick = transl(Tpick, np.array([0.01, 0, 0]))
            Tplace = transl(Tplace, np.array([0.01, 0, 0]))
            it += 1

    if q_pick is None:
        # no grasp, skip
        print "[box %d] is skipped" % i
        env.Remove(box)
        if flag_dest1:
            dest1_boxes_cnt -= 1
        else:
            dest0_boxes_cnt -= 1
        continue


    print "[box %d] [Tpick] IK solution: " % i, q_pick

    # Find q_place
    q_place = manip.FindIKSolution(Tplace, orpy.IkFilterOptions.CheckEnvCollisions) # get collision-free solution
    print "[box %d] [Tplace] IK solution: " % i, q_place

    # Execute pick & place task
    manipprob.MoveManipulator(goal=q_pick, jitter=0.04) # call motion planner with goal joint angles
    robot.WaitForController(0) # wait
    robot.Grab(box)
    
    # traj = manipprob.MoveManipulator(goal=q_place, jitter=0.04, outputtrajobj=True) # call motion planner with goal joint angles
    updir = np.array((0,0,1))
    closedir = np.array((-1, 0, 0))
    sidedir = np.array((0, -1, 0))
    print "[box %d] Pulling close" % i
    for k in range(1):
        manipprob.MoveHandStraight(direction=closedir,stepsize=0.01, minsteps=1, maxsteps=10)
        robot.WaitForController(0)
    print "[box %d] Moving up" % i
    for k in range(30):
        manipprob.MoveHandStraight(direction=updir, stepsize=0.01, minsteps=1, maxsteps=10)
        robot.WaitForController(0)
    print "[box %d] Moving side" % i
    for k in range(30):
        manipprob.MoveHandStraight(direction=sidedir, stepsize=0.01, minsteps=1, maxsteps=10)
        robot.WaitForController(0)
    print "[box %d] Move to place location" % i
    traj = manipprob.MoveManipulator(goal=q_place, jitter=0.04, outputtrajobj=True) # call motion planner with goal joint angles
    robot.WaitForController(0)
    robot.Release(box)
    # switch destination
    flag_dest1 = not flag_dest1
    # raw_input("Press Enter to continue")

    print "[box %d] Compute tilt angle" % i
    print "Tee: \n", manip.GetEndEffectorTransform()
    # Get joints values
    spec = traj.GetConfigurationSpecification()

    times = np.arange(0, traj.GetDuration(), 0.01)
    qvect = np.zeros((len(times), robot.GetActiveDOF()))
    spec = traj.GetConfigurationSpecification()
    for i_time in range(len(times)):
        trajdata = traj.Sample(times[i_time])
        qvect[i_time,:] = spec.ExtractJointValues(trajdata, robot, manip.GetArmIndices(), 0)
    # Compute tilt angle
    angles = np.zeros(len(times))
    with robot:
        for i_time in range(len(times)):
            robot.SetActiveDOFValues(qvect[i_time, :])
            Tee = manip.GetEndEffectorTransform()
            angles[i_time] = np.arccos(Tee[2, 2])
    # plot
    ax.plot(times, angles * 180 / np.pi, label='box %d'%i)


ax.grid(True)
ax.legend()
ax.set_xlabel('Time [s]')
ax.set_ylabel('Tilt angles [deg]')
canvas.print_figure('boxes_tilt_values_3.png')

raw_input("Press Enter to finish")
