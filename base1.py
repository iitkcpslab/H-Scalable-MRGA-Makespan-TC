from glob import glob
import time
from heapq import *
import numpy as np
from math import inf, isinf

from utils.search_methods import DIJKSTRA_2D, DIJKSTRA_3D, FRASTAR, FRASTAR_3D

from utils.class_definitions import Graph
from utils.utility_functions import find_MinVertexCover, get_initial_match, maximize_match, get__threshold_subgraph, update__threshold_subgraph, get__minimal_subgraph, get__equality_subgraph, update__equality_subgraph


left_vertices = set()
right_vertices = set()
# path = {}
# no_of_explorations = 0

# OL_dict = {}
# CL_dict = {}

t1 = 0
t2 = 0
t3 = 0



# ---------------------------------------------------------------------------------------------------------------------

def update_label(G, mvc):
	
	uncovered_robots = left_vertices - mvc
	uncovered_goals = right_vertices - mvc
	
	brk = 0
	delta_min = inf
	for i in uncovered_robots:
		for j in uncovered_goals: 
			if j in G.vertices[i].neighbors:   # redundant IF 
				delta = G.vertices[i].incident_edges[j].weight - ( G.vertices[i].label + G.vertices[j].label )  # aks - MAIN POINT
				# new_alpha = abs(new_alpha) # aks 98
				# if not isinf(delta):
				# delta_min = delta if delta < delta_min else delta_min        # aks - MAIN POINT
				if delta < delta_min:
					delta_min = delta

	
	
	if isinf( delta_min ):
		brk = 1
		return G, brk 


	# if delta_min != None:
	for i in left_vertices & mvc:   # for covered robots
		G.vertices[i].label = G.vertices[i].label - delta_min
	for j in uncovered_goals:       # for uncovered goals
		G.vertices[j].label = G.vertices[j].label + delta_min


	return G, brk


# ---------------------------------------------------------------------------------------------------------------------


# ---------------------------------------------------------------------------------------------------------------------

def update_makespan(G, mvc, makespan):
	
	uncovered_robots = left_vertices - mvc
	uncovered_goals = right_vertices - mvc
	
	deltas = []
	
	for i in uncovered_robots:
		for j in uncovered_goals: 
			# if j in G.vertices[i].neighbors:   # redundant IF 
			edge = G.vertices[i].incident_edges[j]
			# slack = edge.weight - ( G.vertices[i].label + G.vertices[j].label )  # aks - MAIN POINT
			
			# DELTA.append( ( edge.weight, slack ) )  # c77
			deltas.append( edge.weight )  # c77
			

	heapify(deltas) # c77

	
	if makespan < deltas[0]:
		makespan = deltas[0]


	return G, makespan


# ---------------------------------------------------------------------------------------------------------------------



# ---------------------------------------------------------------------------------------------------------------------
# main function starts
# ---------------------------------------------------------------------------------------------------------------------


def find_assignment_N( _G, no_of_robots, no_of_goals ):
	

	global t2, t3
	# no_of_explorations = 0
	# path.clear()
	left_vertices.clear()
	right_vertices.clear()
	
	
	# Construct a bipartite graph 
	
	G = Graph( _G )
	
		
	for x in G.vertices:
		if G.vertices[x].in_left:
			left_vertices.add(x)
			# if x[0] == 'r':
			# 	path[x] = {}
		else:
			right_vertices.add(x)
			# if x[0] == 'r':
			# 	path[x] = {}
	
	
	# Get initial makespan
	
	# rowmin = _G.min(axis=1)
	# colmin = _G.min(axis=0)

	
	if no_of_robots == no_of_goals:
	
		rowmin = _G.min(axis=1) 
		colmin = _G.min(axis=0)

		all_min = np.concatenate( (rowmin, colmin) )
	
	elif no_of_robots < no_of_goals:
		rowmin = _G[:no_of_robots, :no_of_goals].min(axis=1) 
		all_min = rowmin
	
	else:
		colmin = _G[:no_of_robots, :no_of_goals].min(axis=0)
		all_min = colmin


	# makespan = np.max(all_min)
	makespan = np.max(all_min[np.isfinite(all_min)])  # a robot (goal) can be trapped in case of unequal R & G. So, we need to ignore infinite cost for such entity

	# # Generate an initial feasible labeling
	# G.generate_feasible_labeling( left_vertices, right_vertices )

	
	# Get the threshold subgraph
	G_th = get__threshold_subgraph( G, left_vertices, right_vertices, makespan )

	
	# Finding an initial matching
	M = get_initial_match(G_th, left_vertices)

	# Maximize the current matching
	M = maximize_match(G_th, M, left_vertices)

	
	# change 999 - for handling case when no of robots != no of goals
	min_robots_goals = min( len(left_vertices), len(right_vertices) )  # change 999

	
	# while len(M) < int(len(eq_G.vertices)/2):   # change 999
	while len(M) < min_robots_goals:

		mvc = find_MinVertexCover(G_th, M, left_vertices, right_vertices)
		
		G, makespan = update_makespan(G, mvc, makespan)
		
		G_th = update__threshold_subgraph( G_th, G, mvc, left_vertices, right_vertices, makespan )
		
		M = maximize_match(G_th, M, left_vertices)
		


	t2 = time.time()
	# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	# # INVOKE HUNGARIAN METHOD for finding min TC assignment
	# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	# Get minimal subgraph (one which has all edges whose weight <= makespan)

	G_th = get__minimal_subgraph( G_th, G, left_vertices, right_vertices, makespan )
	
	# PROCESS HUNGARIAN STEPS on G_th

	# Generate an initial feasible labeling
	G_th.generate_feasible_labeling( left_vertices, right_vertices )

	# Get the equality subgraph
	eq_G = get__equality_subgraph( G_th, left_vertices, right_vertices )

	# Finding an initial matching
	M = get_initial_match(eq_G, left_vertices)

	# Maximize the current matching
	M = maximize_match(eq_G, M, left_vertices)


	# change 999 - for handling case when no of robots != no of goals
	min_robots_goals = min( len(left_vertices), len(right_vertices) )  # change 999

	
	# while len(M) < int(len(eq_G.vertices)/2):   # change 999
	while len(M) < min_robots_goals:

		mvc = find_MinVertexCover(eq_G, M, left_vertices, right_vertices)    # polynomial time algorithm for finding 'Minimum' vertex cover
		
		
		G_th, brk = update_label(G_th, mvc)
		if brk == 1:
			break
				

		# eq_G = G.equality_subgraph()
		eq_G = update__equality_subgraph( eq_G, G_th, mvc, left_vertices, right_vertices, makespan )
		# M = find_maxMatch(eq_G, left_vertices)
		M = maximize_match(eq_G, M, left_vertices)
		



	
	t3 = time.time()

	# ------------------ End Result -----------------------

	total = 0
	unassigned_count = 0
	assigned_count = 0
	if (len(M) != 0):
		for e in M:
			if not isinf( e.weight ):
				total = total + e.weight
				assigned_count = assigned_count + 1
	

	unassigned_count = min_robots_goals - assigned_count
	

	return list(map(lambda e: ((e.vertices[0], e.vertices[1]), e.weight), M)), total, unassigned_count, makespan
	
# ---------------------------------------------------------------------------------------------------------------------




# --------------------------------------------------------------------------------------------------------------
#                          Directly - exposed function for Naive approach
# --------------------------------------------------------------------------------------------------------------

def naive ( no_of_robots, no_of_goals, workSpace, all_start_loc, all_goal_loc, ws_type, verbosity ):

	global t1, t2, t3
	t1 = time.time()

	path = {}
	# costMatrix = np.zeros((no_of_robots, no_of_goals))
	# For adding dummy robots / goals for the one which is less in number
	max_robots_goals = max( no_of_robots, no_of_goals)  # for dummy 
	costMatrix = np.zeros((max_robots_goals, max_robots_goals))  # for dummy


	total_CL = 0

	# NC 1 # UNCOMMENT below
	# '''

	for i in range(no_of_robots):
		robot_name = 'r' + str(i)
		path[robot_name] = {}
		
		h_cost = 0   # for Dijkstra's algorithm


		# Invoking Dijkstra's algorithm
		if ws_type == '3D':
			# one_path, costMatrix[i][j] = ASTAR_3D( workSpace, all_start_loc[robot_name], all_goal_loc[goal_name], rtype = 'pathAndCost' ) 
			path_dict, cost_dict, CL_count = DIJKSTRA_3D( workSpace, all_start_loc[robot_name], set(all_goal_loc.values()), h_cost, rtype = 'pathAndCost' ) 
		else:
			path_dict, cost_dict, CL_count = DIJKSTRA_2D( workSpace, all_start_loc[robot_name], set(all_goal_loc.values()), h_cost, rtype = 'pathAndCost' )
		
		total_CL = total_CL + CL_count


		
		for j in range(no_of_goals):
			goal_name = 'g' + str(j)
			costMatrix[i][j] = cost_dict[all_goal_loc[goal_name]]
			path[robot_name][goal_name] = path_dict[all_goal_loc[goal_name]]


		if verbosity > 1:
			print( "Baseline case: Paths for robot ", i, " computed" )

		# NC 1 ends
	
	# ''' 
	
	# UNCOMMENT above

	# costMatrix = np.array([
    #         [7, 4, 8, 11],
    #         [7, 4, 6, 11],
    #         [6, 4, 5, 3],
    #         [6, 6, 10, 5]
    #         ])

	# costMatrix = np.array([
    #         [5, 99, 99, 99, 99],
    #         [3, 7, 3, 99, 99],
    #         [6, 99, 2, 99, 4],
    #         [99, 99, 5, 6, 2],
	# 		[99, 4, 99, 1, 6]
    #         ])


	result, total_cost, u_count, makespan = find_assignment_N( costMatrix, no_of_robots, no_of_goals )  
	
	no_of_exp = no_of_robots * no_of_goals

	t_om = t2 - t1
	t_otc = t3 - t2

	return result, total_cost, path, u_count, no_of_exp, makespan, total_CL, t_om, t_otc




# New naive function with FRASTAR

# --------------------------------------------------------------------------------------------------------------
#                          Directly - exposed function for Naive approach
# --------------------------------------------------------------------------------------------------------------

def naive_with_FRASTAR ( no_of_robots, no_of_goals, workSpace, all_start_loc, all_goal_loc, ws_type, verbosity ):

	path = {}
	# costMatrix = np.zeros((no_of_robots, no_of_goals))
	# For adding dummy robots / goals for the one which is less in number
	max_robots_goals = max( no_of_robots, no_of_goals)  # for dummy 
	costMatrix = np.zeros((max_robots_goals, max_robots_goals))  # for dummy

	# global OL_dict, CL_dict
	# OL_dict.clear()
	# CL_dict.clear()


	# '''
	# commented due to NC 1
	for i in range(no_of_robots):
		robot_name = 'r' + str(i)
		path[robot_name] = {}
		# OL_dict[robot_name] = []
		# CL_dict[robot_name] = {}
		OL_dict = []
		CL_dict = {}

		if ws_type == '3D':
			distance_array = np.empty(shape=(workSpace.shape[0], workSpace.shape[1], workSpace.shape[2]))
		else:
			distance_array = np.empty(shape=(workSpace.shape[0], workSpace.shape[1]))
		
		distance_array.fill(inf)



		for j in range(no_of_goals):
			goal_name = 'g' + str(j)
			
			if ws_type == '3D':
				# one_path, costMatrix[i][j] = ASTAR_3D( workSpace, all_start_loc[robot_name], all_goal_loc[goal_name], rtype = 'pathAndCost' ) 
				one_path, costMatrix[i][j], OL_dict, CL_dict, distance_array = FRASTAR_3D( workSpace, all_start_loc[robot_name], all_goal_loc[goal_name], OL_dict, CL_dict, distance_array, rtype = 'pathAndCost' ) 
			else:
				# one_path, costMatrix[i][j], OL_dict[robot_name], CL_dict[robot_name] = FRASTAR( workSpace, all_start_loc[robot_name], all_goal_loc[goal_name], OL_dict[robot_name], CL_dict[robot_name], rtype = 'pathAndCost' )
				one_path, costMatrix[i][j], OL_dict, CL_dict, distance_array = FRASTAR( workSpace, all_start_loc[robot_name], all_goal_loc[goal_name], OL_dict, CL_dict, distance_array, rtype = 'pathAndCost' )
			path[robot_name][goal_name] = one_path
		
		if verbosity > 1:
			print( "Baseline case with FRASTAR: Paths for robot", i,  " computed" )
		# if verbosity > 1:
		# 	print("")

	# '''
	
	

	result, total_cost, u_count, makespan = find_assignment_N( costMatrix )  
	
	no_of_exp = no_of_robots * no_of_goals

	return result, total_cost, path, u_count, no_of_exp, makespan