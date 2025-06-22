# TSWAP - Algo 2's implementation 

import time
from heapq import *
import numpy as np
from math import sqrt, inf, isinf
# from math import inf, isinf
# from operator import itemgetter

from utils.search_methods import FRASTAR, FRASTAR_3D
from utils.class_definitions import Graph, Edge
from utils.utility_functions import find_MinVertexCover, get_initial_match, maximize_match, get__equality_subgraph, update__equality_subgraph


left_vertices = set()
right_vertices = set()
path = {}
no_of_explorations = 0

OL_dict = {}
CL_dict = {}
distance_lookup = {}


t1 = 0
t2 = 0
t3 = 0
t4 = 0

# ---------------------------------------------------------------------------------------------------------------------
# build_queue : Function to collect costs (weights) of edges in a queue
# ---------------------------------------------------------------------------------------------------------------------

def build_queue(G):
	
	cost_queue = []
	for i in left_vertices:
		for j in right_vertices: 
			edge = G.vertices[i].incident_edges[j]
			
			cost_queue.append( ( edge.weight, edge.typeOfWeight, i, j ) )

	
	heapify(cost_queue)
			
	return cost_queue

# **********************************************************************************************************************


def update_label(G, mvc):
	
	uncovered_robots = left_vertices - mvc
	uncovered_goals = right_vertices - mvc
	
	brk = 0
	delta_min = inf
	for i in uncovered_robots:
		for j in uncovered_goals: 
			if j in G.vertices[i].neighbors:   
				delta = G.vertices[i].incident_edges[j].weight - ( G.vertices[i].label + G.vertices[j].label )  # aks - MAIN POINT
				
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
# main function starts
# ---------------------------------------------------------------------------------------------------------------------


def find_assignment_H( _G, workSpace, all_start_loc, all_goal_loc, ws_type ):
	

	global no_of_explorations, distance_lookup, t1, t2, t3, t4, OL_dict, CL_dict
	no_of_explorations = 0
	path.clear()
	left_vertices.clear()
	right_vertices.clear()
	
	OL_dict.clear()
	CL_dict.clear()
	distance_lookup.clear()
	
	
	

	G = Graph( _G, heur = True )
	
		
	for x in G.vertices:
		if G.vertices[x].in_left:
			left_vertices.add(x)
			if x[0] == 'r':
				path[x] = {}
		else:
			right_vertices.add(x)
			if x[0] == 'r':
				path[x] = {}
	


	no_of_robots = len( all_start_loc )
	no_of_goals = len( all_goal_loc )

	min_robots_goals = min( no_of_robots, no_of_goals )


	# initialize the bipartite graph B (here, G_B)

	G_B = Graph( eq_flag = True )  # eq_flag = True means empty bipartite graph with blank set of vertices
	
	# Creating vertices
	for x in left_vertices:
		G_B.add_vertex( x, G.vertices[x].label, G.vertices[x].in_left )

	for y in right_vertices:
		G_B.add_vertex( y, G.vertices[y].label, G.vertices[y].in_left )
	

	# Initialize matching M

	M = set()


	# new

	for x in left_vertices:    
		
		OL_dict[x] = []
		CL_dict[x] = {}
		
		if ws_type == '3D':
			distance_lookup[x] = np.empty(shape=(workSpace.shape[0], workSpace.shape[1], workSpace.shape[2]))
		else:
			distance_lookup[x] = np.empty(shape=(workSpace.shape[0], workSpace.shape[1]))
		
		distance_lookup[x].fill(inf)


	# new ends


	# new - we need to build the queue Q having tuples for all edges 

	cost_queue = build_queue(G)


	# new - start building bipartite graph B

	while(cost_queue):
		
		min_tuple = cost_queue[0]
		heappop(cost_queue)


		if min_tuple[1] == 'h':
			
			# Explore
			CL_countt = 0

			edge = G.vertices[min_tuple[2]].incident_edges[min_tuple[3]]  # edge variable will store the Edge object
			if min_tuple[2][0] == 'r':
				if ws_type == '3D':
					one_path, edge.weight, OL_dict[min_tuple[2]], CL_dict[min_tuple[2]], distance_lookup[min_tuple[2]] = FRASTAR_3D( workSpace, all_start_loc[min_tuple[2]], all_goal_loc[min_tuple[3]], OL_dict[min_tuple[2]], CL_dict[min_tuple[2]], distance_lookup[min_tuple[2]], rtype = 'pathAndCost' )
				else:
					one_path, edge.weight, OL_dict[min_tuple[2]], CL_dict[min_tuple[2]], distance_lookup[min_tuple[2]] = FRASTAR( workSpace, all_start_loc[min_tuple[2]], all_goal_loc[min_tuple[3]], OL_dict[min_tuple[2]], CL_dict[min_tuple[2]], distance_lookup[min_tuple[2]], rtype = 'pathAndCost' )
				path[min_tuple[2]][min_tuple[3]] = one_path
			
			else:
				if ws_type == '3D':
					one_path, edge.weight, OL_dict[min_tuple[2]], CL_dict[min_tuple[2]], distance_lookup[min_tuple[2]] = FRASTAR_3D( workSpace, all_goal_loc[min_tuple[2]], all_start_loc[min_tuple[3]], OL_dict[min_tuple[2]], CL_dict[min_tuple[2]], distance_lookup[min_tuple[2]], rtype = 'pathAndCost' )
				else:
					one_path, edge.weight, OL_dict[min_tuple[2]], CL_dict[min_tuple[2]], distance_lookup[min_tuple[2]] = FRASTAR( workSpace, all_goal_loc[min_tuple[2]], all_start_loc[min_tuple[3]], OL_dict[min_tuple[2]], CL_dict[min_tuple[2]], distance_lookup[min_tuple[2]], rtype = 'pathAndCost' )
				path[min_tuple[3]][min_tuple[2]] = one_path[::-1]
			
			edge.typeOfWeight = 'a'
			no_of_explorations = no_of_explorations + 1


			
			new_tuple = ( edge.weight, 'a', min_tuple[2], min_tuple[3] )

			heappush(cost_queue, new_tuple)

			continue


		# add new edge with actual cost to the bipartite graph B
		edge = G.vertices[min_tuple[2]].incident_edges[min_tuple[3]]  # edge variable will store the Edge object

		G_B.vertices[min_tuple[2]].neighbors.add(min_tuple[3])
		G_B.vertices[min_tuple[3]].neighbors.add(min_tuple[2])
		G_B.vertices[min_tuple[2]].incident_edges[min_tuple[3]] = edge
		G_B.vertices[min_tuple[3]].incident_edges[min_tuple[2]] = edge

		makespan = edge.weight


		# Update matching 
		M = maximize_match(G_B, M, left_vertices)

		# if len(M) == no_of_goals:
		if len(M) == min_robots_goals:
			break


	# Hungarian method - about to start

	
	t2 = time.time()

	
	# Save a copy of nexp until now, so that it will represent # exp needed to get optimal makespan (excluding min TC)
	makespan_nexp = no_of_explorations
	
	
	# counting number of nodes expanded (for optimal makespan only - excluding Hungarian)
	makespan_CL_tswap = 0

	for robo in CL_dict:
		makespan_CL_tswap = makespan_CL_tswap + len(CL_dict[robo])

	
	t3 = time.time()
	
	
	# Add remaining edges to B whose cost in Q are equal to the makespan value 

	#qbegins

	while( ( cost_queue ) and ( cost_queue[0][0] == makespan ) ):
		
		min_tuple = cost_queue[0]
		heappop(cost_queue)


		if min_tuple[1] == 'h':
			
			# Explore
			CL_countt = 0

			edge = G.vertices[min_tuple[2]].incident_edges[min_tuple[3]]  # edge variable will store the Edge object
			if min_tuple[2][0] == 'r':
				if ws_type == '3D':
					one_path, edge.weight, OL_dict[min_tuple[2]], CL_dict[min_tuple[2]], distance_lookup[min_tuple[2]] = FRASTAR_3D( workSpace, all_start_loc[min_tuple[2]], all_goal_loc[min_tuple[3]], OL_dict[min_tuple[2]], CL_dict[min_tuple[2]], distance_lookup[min_tuple[2]], rtype = 'pathAndCost' )
				else:
					one_path, edge.weight, OL_dict[min_tuple[2]], CL_dict[min_tuple[2]], distance_lookup[min_tuple[2]] = FRASTAR( workSpace, all_start_loc[min_tuple[2]], all_goal_loc[min_tuple[3]], OL_dict[min_tuple[2]], CL_dict[min_tuple[2]], distance_lookup[min_tuple[2]], rtype = 'pathAndCost' )
				path[min_tuple[2]][min_tuple[3]] = one_path
			
			else:
				if ws_type == '3D':
					one_path, edge.weight, OL_dict[min_tuple[2]], CL_dict[min_tuple[2]], distance_lookup[min_tuple[2]] = FRASTAR_3D( workSpace, all_goal_loc[min_tuple[2]], all_start_loc[min_tuple[3]], OL_dict[min_tuple[2]], CL_dict[min_tuple[2]], distance_lookup[min_tuple[2]], rtype = 'pathAndCost' )
				else:
					one_path, edge.weight, OL_dict[min_tuple[2]], CL_dict[min_tuple[2]], distance_lookup[min_tuple[2]] = FRASTAR( workSpace, all_goal_loc[min_tuple[2]], all_start_loc[min_tuple[3]], OL_dict[min_tuple[2]], CL_dict[min_tuple[2]], distance_lookup[min_tuple[2]], rtype = 'pathAndCost' )
				path[min_tuple[3]][min_tuple[2]] = one_path[::-1]
			
			edge.typeOfWeight = 'a'
			no_of_explorations = no_of_explorations + 1

			

			
			new_tuple = ( edge.weight, 'a', min_tuple[2], min_tuple[3] )

			heappush(cost_queue, new_tuple)

			continue


		# add new edge with actual cost to the bipartite graph B
		edge = G.vertices[min_tuple[2]].incident_edges[min_tuple[3]]  # edge variable will store the Edge object

		G_B.vertices[min_tuple[2]].neighbors.add(min_tuple[3])
		G_B.vertices[min_tuple[3]].neighbors.add(min_tuple[2])
		G_B.vertices[min_tuple[2]].incident_edges[min_tuple[3]] = edge
		G_B.vertices[min_tuple[3]].incident_edges[min_tuple[2]] = edge



	# qends 

	# PROCESS HUNGARIAN STEPS on G_B
	
	# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	# # INVOKING HUNGARIAN METHOD for finding min TC assignment
	# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


	# Generate an initial feasible labeling
	G_B.generate_feasible_labeling( left_vertices, right_vertices )

	# Get the equality subgraph
	eq_G = get__equality_subgraph( G_B, left_vertices, right_vertices )

	# Finding an initial matching
	M = get_initial_match(eq_G, left_vertices)

	# Maximize the current matching
	M = maximize_match(eq_G, M, left_vertices)


	
	min_robots_goals = min( len(left_vertices), len(right_vertices) )  

	
	while len(M) < min_robots_goals:

		mvc = find_MinVertexCover(eq_G, M, left_vertices, right_vertices)
		
		
		G_B, brk = update_label(G_B, mvc)
		if brk == 1:
			break
				

		# eq_G = G.equality_subgraph()
		eq_G = update__equality_subgraph( eq_G, G_B, mvc, left_vertices, right_vertices, makespan )
		# M = find_maxMatch(eq_G, left_vertices)
		M = maximize_match(eq_G, M, left_vertices)
	

	# paste ends
	
	t4 = time.time()

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

	# counting number of nodes expanded BEGINS (for OM+OTC)
	total_CL_tswap = 0

	for robo in CL_dict:
		total_CL_tswap = total_CL_tswap + len(CL_dict[robo])
	
		
	
	return list(map(lambda e: ((e.vertices[0], e.vertices[1]), e.weight), M)), total, path, unassigned_count, makespan_nexp, no_of_explorations, makespan, makespan_CL_tswap, total_CL_tswap
	
# **********************************************************************************************************************



# --------------------------------------------------------------------------------------------------------------
#                          Directly - exposed function for Heuristic-based approach
# --------------------------------------------------------------------------------------------------------------

def heuristic_tswap ( no_of_robots, no_of_goals, workSpace, all_start_loc, all_goal_loc, ws_type, verbosity  ):


	global t1, t2, t3, t4
	
	t1 = time.time()

	costMatrix = np.zeros((no_of_robots, no_of_goals))
	# Generating heuristic costs for all pairs of robots and goals
	for i in range(no_of_robots):
		robot_name = 'r' + str(i)
		for j in range(no_of_goals):
			goal_name = 'g' + str(j)
			
			if ws_type == '3D':
				sq_of_distance = ((all_start_loc[robot_name][0] - all_goal_loc[goal_name][0]) ** 2) + (
				(all_start_loc[robot_name][1] - all_goal_loc[goal_name][1]) ** 2) + (
					(all_start_loc[robot_name][2] - all_goal_loc[goal_name][2]) ** 2)
			else:
				sq_of_distance = ((all_start_loc[robot_name][0] - all_goal_loc[goal_name][0]) ** 2) + (
					(all_start_loc[robot_name][1] - all_goal_loc[goal_name][1]) ** 2)

			# costMatrix[i][j] = float(format(sqrt(sq_of_distance), '.2f'))      
			costMatrix[i][j] = sqrt(sq_of_distance)
			

		
	result_h, total_cost_h, path_h, u_count_h, makespan_nexp_h, no_of_exp_h, makespan, makespan_CL_tswap, total_CL_tswap = find_assignment_H( costMatrix, workSpace, all_start_loc, all_goal_loc, ws_type )

	# makespan_CL_tswap is CL count for just OM only, while total_CL_tswap is CL count for OM+OTC

	if result_h is None:
		return None, None, None, None, None, None, None, None, None, None, None

	
	t_om = t2 - t1
	t_otc = t4 - t3

	return result_h, total_cost_h, path_h, u_count_h, makespan_nexp_h, no_of_exp_h, makespan, makespan_CL_tswap, total_CL_tswap, t_om, t_otc


# **********************************************************************************************************************
