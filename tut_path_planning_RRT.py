#!/usr/bin/env python
import numpy as np
import pylab as pl
import sys


sys.path.append('~/ros/src/cri/osr_examples/scripts/')
import environment_2d
pl.ion()
np.random.seed(4)
env = environment_2d.Environment(10, 6, 5)
pl.clf()
env.plot()
q = env.random_query()
if q is not None:
	x_start, y_start, x_goal, y_goal = q
	env.plot_query(x_start, y_start, x_goal, y_goal)


class Node(object):
	"""docstring for Node"""
	def __init__(self, _data):
		self.left = []
		self.right = []
		self.data = _data
	
	def addChild(self, childs_list):
		for child in childs_list:
			assert isinstance(child, Node)
			if child.data[0] > self.data[0]:
				# child on the right of this node
				self.right.append(child)
			else:
				self.left.append(child)

	def dist(self, q):
		'''
		Square distance betweeen this node and target coordinate (q) 
		'''
		return (self.data[0] - q[0])**2 + (self.data[1] - q[1])**2

	def onNodeLeft(self, q):
		return q[0] < self.data[0]


class Tree(object):
	"""docstring for Tree"""
	def __init__(self, _root, _nodes_list=[]):
		assert _root in _nodes_list
		self.nodes_list = _nodes_list
		self.root = _root

	def addNode(self, _node):
		assert isinstance(_node, Node)
		self.nodes_list.append(_node)

	def addEdge(self, parent, child):
		if child.data[0] < parent.data[0]:
			# child on the left of parent
			parent.left.append(child)
		else:
			parent.right.append(child)

	def nearestNeighbor(self, q, epsilon=0.5):
		open_set = [self.root]
		min_dist = 1e5
		reached = False
		node_nearest = None
		while len(open_set) > 0 and not reached:
			node = open_set.pop()
			d = node.dist(q) 
			if d < min_dist:
				min_dist = d
				node_nearest = node
			
			if min_dist < epsilon**2:
				print "Bingoooo! Reached q"
				reached = True
			
			if node.onNodeLeft(q):
				open_set += node.left
			else:
				open_set += node.right
		return node_nearest  # this node has nothing but data (i.e. no parent, no child)

	def display(self):
		open_set = [self.root]
		while len(open_set) > 0:
			node = open_set.pop()
			pl.plot([node.data[0]], [node.data[1]], 'go', markersize=8)
			childs_list = node.left + node.right
			for child in childs_list:
				pl.plot([node.data[0], child.data[0]], [node.data[1], child.data[1]], 'k-',linewidth=2)
			open_set += childs_list

	def steer(self, node_nearest, q, env, r=0.5):
		def lineDistSample(v1, v2, alpha):
			def sign(x):
				if x > 0:
					return 1
				return -1
			length = np.sqrt((v1[0] - v2[0])**2 + (v1[1] - v2[1])**2)
			if abs(v2[1] - v1[1]) < 1e-5:
				x = v1[0] + alpha * length * sign(v2[0] - v1[0])
				y = v1[1]
			else:
				gamma = (v2[0] - v1[0]) / (v2[1] - v1[1])
				y = v1[1] + alpha * length / np.sqrt(gamma**2 + 1) * sign(v2[1] - v1[1])
				x = v1[0] + gamma * (y - v1[1])
			return x, y
		def lineCheck(v1, v2, env, num_segments=50):
			'''
			return True edge{v1, v2} doesn't cross any obstacle
			'''
			cross_obs = False
			alpha = 1./num_segments
			step = 1./num_segments
			while alpha < 1 and not cross_obs:
				x, y = lineDistSample(v1, v2, alpha)
				if env.check_collision(x, y):
					cross_obs = True
				else:
					alpha += step
			return not cross_obs

		d = node_nearest.dist(q)
		status = ''
		if d < r:
			node_new = Node(q)
			status = 'reached'
		else:
			x, y = lineDistSample(node_nearest.data, q, r*1./d)
			node_new = Node((x, y))
			status = 'advanced'
		# check if line segment from node_nearest to node_new is collision free
		if not lineCheck(node_nearest.data, q, env):
			status = 'trapped'

		return status, node_new


	def expand(self, q):
		node_nearest = self.nearestNeighbor(q)
		pass
		
	def connectPlanner(self):
		pass
		


x_size = 10
y_size = 6
start = (x_start, y_start)
q_rand = (np.random.rand() * x_size, np.random.rand() * y_size)


# create a tree
# root = Node(start)
root = Node((2.5, 5.2))
n1 = Node((1.7, 5.0))
n2 = Node((2.0, 4.5))
n3 = Node((3., 4.75))
n4 = Node((1.85, 4.3))
n5 = Node((3., 4.2))
# q_rand = (2.2, 3.8)
root.addChild([n1, n2, n3])
n2.addChild([n4, n5])

tree = Tree(root, [root, n1, n2, n3, n4, n5])
node_nearest = tree.nearestNeighbor(q_rand)
status, node_new = tree.steer(node_nearest, q_rand, env) 

print "start = ", start
print "rand config = ", q_rand
pl.plot([q_rand[0]], [q_rand[1]], 'g*', markersize=8)

raw_input("Press Enter to display tree")
tree.display()

raw_input("Press Enter to display node_new")
pl.plot([node_nearest.data[0]], [node_nearest.data[1]], 'yo', markersize=8)
print "status: ", status
pl.plot([node_new.data[0]], [node_new.data[1]], 'b*', markersize=12)
raw_input("Press Enter to finish")
