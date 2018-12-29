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
	
# Construct roadmap
num_nodes = 300
# num_neighbors = 10
V = []  # set of vertices
E = []  # set of edges
size_x = 10
size_y = 6

# Construct V
while len(V) < num_nodes:
	found_q = False
	while not found_q:
		x = np.random.rand() * size_x
		y = np.random.rand() * size_y
		if not env.check_collision(x, y):
			found_q = True
			V.append((x, y))

V.sort(key=lambda tup:tup[1])  # sort V based on y


def dist(v1, v2, manhattan=False):
	if manhattan:
		return abs(v2[0] - v1[0]) + abs(v2[1] - v1[1])

	return (v2[0] - v1[0])**2 + (v2[1] - v1[1])**2 


def sign(x):
	if x > 0:
		return 1.
	else:
		return -1.


def lineDistSample(v1, v2, alpha):
	length = np.sqrt(dist(v1, v2))
	if abs(v2[1] - v1[1]) < 1e-5:
		x = v1[0] + alpha * length * sign(v2[0] - v1[0])
		y = v1[1]
	else:
		gamma = (v2[0] - v1[0]) / (v2[1] - v1[1])
		y = v1[1] + alpha * length / np.sqrt(gamma**2 + 1) * sign(v2[1] - v1[1])
		x = v1[0] + gamma * (y - v1[1])
	return x, y


def edgeCheck(v1, v2, num_segments=50):
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


def connect2Dest(dest, V, num_neighbors=20):
	'''
	Connect the goal (or starting point) to the closest node in V 
	'''
	min_dist = 1e5
	closest_node = ()
	checked_node = 0
	for v in V:
		d = dist(v, dest)
		if d > 0 and edgeCheck(v, dest) and d < min_dist:
			closest_node = v
			checked_node += 1
		if checked_node > num_neighbors:
			break

	return closest_node


# Construct E
r = 1.
cross_edges = []
for v in V:
	for v1 in V:
		d = dist(v, v1)
		if d < r**2 and d > 0:
			if edgeCheck(v, v1) and {v, v1} not in E:
				E.append({v, v1})

# Connect start and goal to V
start = (x_start, y_start)
goal = (x_goal, y_goal)
start_neighbor = connect2Dest(start, V)
goal_neighbor = connect2Dest(goal, V)
if start_neighbor == () or goal_neighbor == ():
	print "Failed to connect start or goal to V"
	exit()
else:
	E.append({start, start_neighbor})
	E.append({goal, goal_neighbor})

# Transform E to dictionary, key = vertex, value = vertecies connected to key
graph = {}
graph[start] = []
graph[goal] = []
for v in V:
	graph[v] = []
for e in E:
	tup_e = tuple(e)
	graph[tup_e[0]].append(tup_e[1])
	graph[tup_e[1]].append(tup_e[0])

# A*
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

# Display vertices
for ver in V:
	pl.plot([ver[0]], [ver[1]], marker='o', markersize=4, color="blue")

# Display edges
for edge in E:
	edge_ = tuple(edge)
	v0 = edge_[0]
	v1 = edge_[1]
	pl.plot([v0[0], v1[0]], [v0[1], v1[1]], '-', color="green")

raw_input('Press Enter run A*')



# initialize cost for every node
cost_rec = {}  # list of (vertex, cost)
for v in graph.keys():
	if v == start:
		cost_rec[v] = 0
	else:
		cost_rec[v] = 1e5

open_set = PriorityQueue()
open_set.insert((start, 0))

reach = False
predecessor = {}
while not open_set.isEmpty() and not reach:
	vertex_cost = open_set.dequeue()
	v = vertex_cost[0]
	cost = vertex_cost[1]
	if v == goal:
		reach = True
	else:
		for v_prime in graph[v]:
			cost_prime = cost + dist(v_prime, goal, manhattan=True)
			if cost_prime < cost_rec[v_prime]: 
				cost_rec[v_prime] = cost_prime
				predecessor[v_prime] = v
				open_set.insert((v_prime, cost_prime))
if reach:
	print "Bingooooooooooo"
	raw_input("Press Enter to display path")
	# display path
	reach_start = False
	v_prime = goal
	while not reach_start:
		v = predecessor[v_prime]
		pl.plot([v_prime[0], v[0]], [v_prime[1], v[1]], color="blue", linewidth=2)
		if v == start:
			reach_start = True
		else:
			v_prime = v
else:
	print "Path not found"


raw_input("Press Enter to finish")
