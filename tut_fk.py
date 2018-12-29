#!/usr/bin/env python
import openravepy as orpy
import numpy as np
from scipy.linalg import expm


def skew(ome):
	return np.array([[0, -ome[2], ome[1]],
					[ome[2], 0, -ome[0]],
					[-ome[1], ome[0], 0]])


env = orpy.Environment()
env.Load('/home/mquan/ros/src/cri/osr_course_pkgs/osr_openrave/robots/denso_robotiq_85_gripper.robot.xml')
env.SetDefaultViewer()
robot = env.GetRobot('denso_robotiq_85_gripper')
manipulator = robot.SetActiveManipulator('gripper')
robot.SetActiveDOFs(manipulator.GetArmIndices())


# Improve the visualization settings
np.set_printoptions(precision=6, suppress=True)

# Exercise
q1 = np.array([-0.1, 1.8, 1.0, 0.5, 0.2, 1.3])
qdot1 = np.array([1.2, -0.7, -1.5, -0.5, 0.8, -1.5])
delta_t = 0.1

# Starting pose
robot.SetActiveDOFValues(q1)

link_idx = [l.GetName() for l in robot.GetLinks()].index('robotiq_85_base_link')
link_origin = robot.GetLink('robotiq_85_base_link').GetTransform()[:3,3]

T1 = robot.GetLink('robotiq_85_base_link').GetTransform()

J_lin = robot.ComputeJacobianTranslation(link_idx, link_origin)
J_ang = robot.ComputeJacobianAxisAngle(link_idx)

v = np.dot(J_lin[:, :6], qdot1.reshape((6, 1)))
omega = np.dot(J_ang[:, :6], qdot1.reshape((6, 1)))

x2 = T1[:3, -1].reshape(v.shape) + delta_t * v
R2 = np.dot(expm(delta_t * skew(omega)), T1[:3, :3])
T2 = np.concatenate((R2, x2), axis=1)

# Move to new pose
robot.SetActiveDOFValues(q1 + delta_t * qdot1)
abs_T2 = robot.GetLink('robotiq_85_base_link').GetTransform() 

print "Difference"
print abs_T2[:3, :] - T2
