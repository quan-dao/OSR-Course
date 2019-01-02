import numpy as np
import openravepy as orpy
from robot_wrapper_class import RobotWrapper, box2Ttarget

np.random.seed(4)

env = orpy.Environment()
env.Load('/home/mquan/ros/src/cri/osr_course_pkgs/osr_openrave/worlds/pick_and_place.env.xml')
env.SetViewer('qtcoin')
# env.SetDefaultViewer()
# robot = env.GetRobot("Denso")
# # print robot
# manipulator = robot.SetActiveManipulator('gripper')
# robot.SetActiveDOFs(manipulator.GetArmIndices())

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

# Checkout box transform
# box = boxes[0]
# print box.GetTransform()

# Sort boxes according to height
boxes.sort(key=lambda box:box.GetTransform()[2, 3], reverse=True)
# for i, box in enumerate(boxes):
#     print "box %d\t" % i, box.ComputeAABB().pos()

# create the robot object
robot = RobotWrapper("Denso", env)

h_offset = 0.011
delta_h_offset = 0.011
# move first box to destination0
# Ttarget_grab = box2Ttarget(boxes[0])
# Ttarget_release = box2Ttarget(destination0, False, h_offset)

# raw_input("=================displat boxes[0]")
# box = boxes[0]
# color = [0.6, 0.6, 0]
# g = box.GetLinks()[0].GetGeometries()[0]
# g.SetAmbientColor(color)
# g.SetDiffuseColor(color)
# print "box transform: \n", box.GetTransform()
# # robot.testTtarget(Ttarget_grab)
# raw_input("wait")
# move to box i
# robot.move2Ttarget(Ttarget_grab)
# # pick up box i
# robot.pickUp(boxes[i])
# # release at destination
# robot.move2Ttarget(Ttarget_release)
# robot.putDown(boxes[i])


raw_input("Press Enter to start")
for i in range(20):
    box = boxes[i]
    # create Ttarget
    Ttarget_grab = box2Ttarget(box)
    Ttarget_release = box2Ttarget(destination1, False, h_offset)
    # move to box
    robot.move2Ttarget(Ttarget_grab)
    # pick up box
    robot.pickUp(box)
    # release at destination
    robot.move2Ttarget(Ttarget_release)
    robot.putDown(box)
    # increase h
    h_offset += delta_h_offset


    raw_input("=================displat boxes[0]")
    color = [0.6, 0.6, 0]
    g = box.GetLinks()[0].GetGeometries()[0]
    g.SetAmbientColor(color)
    g.SetDiffuseColor(color)
    print "box transform: \n", box.GetTransform()

    g = boxes[i+1].GetLinks()[0].GetGeometries()[0]
    g.SetAmbientColor(color)
    g.SetDiffuseColor(color)
    
    raw_input("Press Enter to pick up next box")



raw_input("Press Enter to finish")