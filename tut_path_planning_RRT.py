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
		assert isinstance(_root, Node)
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
				# print "Bingoooo! Reached q"
				reached = True
			
			if node.onNodeLeft(q):
				open_set += node.left
			else:
				open_set += node.right
		return node_nearest  # this node has nothing but data (i.e. no parent, no child)

	def display(self, node_color='green'):
		open_set = [self.root]
		while len(open_set) > 0:
			node = open_set.pop()
			if node.data != self.root.data:
				pl.plot([node.data[0]], [node.data[1]], 'o', color=node_color, markersize=8)
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

		if env.check_collision(q[0], q[1]):
			return 'trapped', None

		d = node_nearest.dist(q)
		if d < r and lineCheck(node_nearest.data, q, env):
			status = 'reached'
			node_new = Node(q)
		else:
			beta = 1./10
			step = beta
			while beta < 1.:
				# sampled
				x_new, y_new = lineDistSample(node_nearest.data, q, beta)
				if not lineCheck(node_nearest.data, (x_new, y_new), env):
					beta -= step  # back to previous sample
					break
				else:
					beta += step
			if beta == 0.:
				status = 'trapped'
				node_new = None
			else:
				status = 'advanced'
				x_new, y_new = lineDistSample(node_nearest.data, q, beta)
				node_new = Node((x_new, y_new))
		return status, node_new
		'''
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
		'''
	
	def extend(self, q, env):
		node_nearest = self.nearestNeighbor(q)
		status, node_new = self.steer(node_nearest, q, env)
		if status is not 'trapped':
			self.addNode(node_new)
			self.addEdge(node_nearest, node_new)
		return status
		
	def toCoordGraph(self):
		graph = {}
		for node in self.nodes_list:
			graph[node.data] = []
			childs_list = node.left + node.right
			for child in childs_list:
				graph[node.data].append(child.data)
		return graph


class rrtConnectPlanner(object):
	"""docstring for rrtConnectPlanner"""
	def __init__(self, _start_coord, _goal_coord, _env):
		self.start = Node(_start_coord)
		self.goal = Node(_goal_coord)
		self.env = _env
 
	def connect(self, tree_a, q):
		assert isinstance(tree_a, Tree)
		status = 'advanced'
		while status == 'advanced':
			status = tree_a.extend(q, env)
			print '[connect] status = ', status,
		return status

	def connectPlanner(self):
		tree_a = Tree(self.start, [self.start])
		tree_b = Tree(self.goal, [self.goal])
		trees_list = [tree_a, tree_b]
		i = True
		x_size = self.env.size_x
		y_size = self.env.size_y
		max_iter = 1200
		for k in range(max_iter):
			print "\niter %d:" % k
			tree_a = trees_list[i]
			tree_b = trees_list[not i]
			q_rand = (np.random.rand() * x_size, np.random.rand() * y_size)
			if not tree_a.extend(q_rand, self.env) == 'trapped':
				q_new = tree_a.nodes_list[-1].data
				connect_status = tree_b.extend(q_new, self.env)
				print '[connect] status = ', connect_status,
				if connect_status == 'reached':
					return self.path(tree_a, tree_b)
			# swap role of tree_a and tree_b
			i = not i

		tree_a.display('green')
		tree_b.display('yellow')

	def path(self, tree_a, tree_b):
		print "Bingoooo, 2 tree are connected"
		print "tree a 's end node = ", tree_a.nodes_list[-1].data
		print "tree b 's end node = ", tree_b.nodes_list[-1].data
		tree_a.display('green')
		tree_b.display('yellow')
		a_start_a = AStar(tree_a.toCoordGraph(), tree_a.root.data, tree_a.nodes_list[-1].data)
		a_start_b = AStar(tree_b.toCoordGraph(), tree_b.root.data, tree_b.nodes_list[-1].data)
		a_start_a.constructPath()
		a_start_b.constructPath()


class PriorityQueue(object):
	def __init__(self):
		self.queue = []

	def insert(self, item):
		'''
		item = (vertex, cost)
		'''
		self.queue.append(item)

	def isEmpty(self):
		return len(self.queue) == 0

	def dequeue(self):
		try:
			min_cost = 1e5
			target = 0
			for i, item in enumerate(self.queue):
				if item[1] < min_cost:
					min_cost = item[1]
					target = i
			dequeued_item = self.queue[i] 
			del self.queue[i]
			return dequeued_item
		except Exception as e:
			raise e


class AStar(object):
	"""docstring for AStar"""
	def __init__(self, _graph, _start_coord, _goal_coord):
		assert isinstance(_graph, dict)
		assert isinstance(_start_coord, tuple)
		assert isinstance(_goal_coord, tuple)
		self.graph = _graph
		self.cost_rec = {}  # list of (vertex, cost)
		for v in self.graph.keys():
			if v == start:
				self.cost_rec[v] = 0
			else:
				self.cost_rec[v] = 1e5
		self.predecessor = {}  # store element to construct path
		self.start_coord = _start_coord
		self.goal_coord = _goal_coord

	def dist(self, v1, v2, manhattan=True):
		return abs(v1[0] - v2[0]) + abs(v1[1] - v2[1])

	def constructPath(self):
		open_set = PriorityQueue()
		open_set.insert((self.start_coord, 0))

		reach = False
		while not open_set.isEmpty() and not reach:
			vertex_cost = open_set.dequeue()
			v = vertex_cost[0]
			cost = vertex_cost[1]
			if v == self.goal_coord:
				reach = True
			else:
				for v_prime in self.graph[v]:
					cost_prime = cost + self.dist(v_prime, self.goal_coord)
					if cost_prime < self.cost_rec[v_prime]: 
						self.cost_rec[v_prime] = cost_prime
						self.predecessor[v_prime] = v
						open_set.insert((v_prime, cost_prime))
		if reach:
			print "Found a path !!!"
			self.displayPath()
		else:
			print "Path not found"

	def displayPath(self):
		raw_input("Press Enter to display path")
		# display path
		reach_start = False
		v_prime = self.goal_coord
		while not reach_start:
			try:
				v = self.predecessor[v_prime]
				pl.plot([v_prime[0], v[0]], [v_prime[1], v[1]], color="blue", linewidth=2)
			except Exception as e:
				raw_input("Encounter error, press Enter to escape")
				raise e
			# v = self.predecessor[v_prime]
			# pl.plot([v_prime[0], v[0]], [v_prime[1], v[1]], color="blue", linewidth=2)
			if v == start:
				reach_start = True
			else:
				v_prime = v
			
						

'''
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
'''

start = (x_start, y_start)
goal = (x_goal, y_goal)

raw_input("Press Enter to find path")

path_planner = rrtConnectPlanner(start, goal, env) 
path_planner.connectPlanner()

raw_input("Press Enter to finish")
