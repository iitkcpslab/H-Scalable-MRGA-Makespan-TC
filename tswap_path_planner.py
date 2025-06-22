import time
# from heapq import *
import numpy as np
from math import sqrt, inf, isinf

from utils.search_methods import ASTAR, ASTAR_3D


t1 = 0
t2 = 0


# --------------------------------------------------------------------------------------------------------------
#                          TSWAP's Path Planning - Directly - exposed function
# --------------------------------------------------------------------------------------------------------------

# def heuristic ( no_of_robots, no_of_goals, workSpace, all_start_loc, all_goal_loc, ws_type ):
def tswap_path_planning ( result_h, path_h, workSpace, all_start_loc, all_goal_loc, ws_type ):


	global t1, t2
	

	agents_reached_goal = 0
	path_cfree = {}
	current_goal = {}
	individual_path_cost = {}
	all_path_cost = 0
	
	
	# creating new path repository - it stores paths location-wise instead of robot-goal names-wise
	path_all = {}
	
	for goal in all_goal_loc:
			path_all[all_goal_loc[goal]] = {}

	
	for robot in path_h:
		for goal in path_h[robot]:
			path_all[all_goal_loc[goal]][all_start_loc[robot]] = path_h[robot][goal]


	for res in result_h:
		individual_path_cost[res[0][0]] = []
		individual_path_cost[res[0][0]].append(0)


	min_robots_goals = min( len(all_start_loc), len(all_goal_loc) )  # change 999

	
	t1 = time.time()

	# just initialization
	for res in result_h:
		path_cfree [ res[0][0] ] = []
		path_cfree [ res[0][0] ].append( all_start_loc[ res[0][0] ] )    # this step initiates building the collision-free paths
		current_goal[res[0][0]] = res[0][1]


	
	planned = set()

	# main logic
	# while agents_reached_goal < len(all_start_loc):
	while agents_reached_goal < min_robots_goals:
		agents_reached_goal = 0
		planned.clear()
		
		# for each agent
		for res in result_h:
			
			x = res[0][0]
			y = current_goal[x]
			
			planned.add(x)
			
			agent_curr_loc = path_cfree[x][-1]

			
			if x[0] == 'r':
				agent_goal_loc = all_goal_loc[ y ]
			else:
				agent_goal_loc = all_start_loc[ y ]

			
			if ( agent_curr_loc == agent_goal_loc ):
				agents_reached_goal = agents_reached_goal + 1
				continue

			# else:

			# get next Node u for the agent x under consideration
			# check if we already have a path from current location to the goal location:
			found = 0
			
			# e_loc = end location of a path, s_loc = start location of a path
			e_loc = agent_goal_loc

			for s_loc in path_all[e_loc]:
				if agent_curr_loc in path_all[e_loc][s_loc]:
					# no need of A-star search
					index = path_all[e_loc][s_loc].index(agent_curr_loc)
					u = path_all[e_loc][s_loc][index+1]   # u is a location coordinate
					found = 1
					break

			if found == 0:  # perform A-star search and save the path in the repository path_all; get value for u
				# performing A-star search
				if x[0] == 'r':
					# Explore A.cost
					if ws_type == '3D':
						# one_path, edge.weight = ASTAR_3D( workSpace, all_start_loc[x], all_goal_loc[y], rtype = 'pathAndCost' )
						one_path, weight, CL_count = ASTAR_3D( workSpace, agent_curr_loc, agent_goal_loc, rtype = 'pathAndCost' )
					else:
						# one_path, edge.weight = ASTAR( workSpace, all_start_loc[x], all_goal_loc[y], rtype = 'pathAndCost' )
						one_path, weight, CL_count = ASTAR( workSpace, agent_curr_loc, agent_goal_loc, rtype = 'pathAndCost' )
					path_all[agent_goal_loc][agent_curr_loc] = one_path
				else:
					if ws_type == '3D':
						# one_path, edge.weight = ASTAR_3D( workSpace, all_start_loc[y], all_goal_loc[x], rtype = 'pathAndCost' )
						one_path, weight, CL_count = ASTAR_3D( workSpace, agent_goal_loc, agent_curr_loc, rtype = 'pathAndCost' )
					else:
						# one_path, edge.weight = ASTAR( workSpace, all_start_loc[y], all_goal_loc[x], rtype = 'pathAndCost' )
						# one_path, edge.weight, OL_dict[x], CL_dict[x] = FRASTAR( workSpace, all_start_loc[y], all_goal_loc[x], OL_dict[x], CL_dict[x], rtype = 'pathAndCost' )
						one_path, weight, CL_count = ASTAR( workSpace, agent_goal_loc, agent_curr_loc, rtype = 'pathAndCost' )
					path_all[agent_goal_loc][agent_curr_loc] = one_path[::-1]

				u = one_path[1]  # taking index = 1 position in the path as the next position; index = 0 is agent's current loc

			# u is found, now next step from algo is to check if some other robot b is already sitting on u
			current_loc_collision = 0
			for res in result_h:
				b = res[0][0]
				
				bv = path_cfree[b][-1]  # it is a location coordinate 

				bg = all_goal_loc[ current_goal[b] ]   # it is a location coordinate 

				if x != b and u == bv and (b in planned):
					current_loc_collision = 1

					# there will be a STAY motion for agent x due to agent b
					path_cfree[x].append(agent_curr_loc)
					individual_path_cost[x].append( individual_path_cost[b][-1] )

					if u == bg:   # swap goals
						temp = current_goal[x]
						current_goal[x] = current_goal[b]
						current_goal[b] = temp

					else: 
						# detect & resolve deadlock
						A_prime = {x}
						while(True):
							if bv == bg:
								break
							
							# get next node w in path from bv to bg
							found = 0
							for s_loc in path_all[bg]:
								if bv in path_all[bg][s_loc]:
									# no need of A-star search
									index = path_all[bg][s_loc].index(bv)
									w = path_all[bg][s_loc][index+1]   # u is a location coordinate
									found = 1
									break

							if found == 0:  # perform A-star search and save the path in the repository path_all; get value for u
								# performing A-star
								if b[0] == 'r':
									# Explore A.cost
									if ws_type == '3D':
										# one_path, edge.weight = ASTAR_3D( workSpace, all_start_loc[x], all_goal_loc[y], rtype = 'pathAndCost' )
										one_path, weight, CL_count = ASTAR_3D( workSpace, bv, bg, rtype = 'pathAndCost' )
									else:
										# one_path, edge.weight = ASTAR( workSpace, all_start_loc[x], all_goal_loc[y], rtype = 'pathAndCost' )
										one_path, weight, CL_count = ASTAR( workSpace, bv, bg, rtype = 'pathAndCost' )
									path_all[bg][bv] = one_path
								else:
									if ws_type == '3D':
										# one_path, edge.weight = ASTAR_3D( workSpace, all_start_loc[y], all_goal_loc[x], rtype = 'pathAndCost' )
										one_path, weight, CL_count = ASTAR_3D( workSpace, bg, bv, rtype = 'pathAndCost' )
									else:
										# one_path, edge.weight = ASTAR( workSpace, all_start_loc[y], all_goal_loc[x], rtype = 'pathAndCost' )
										# one_path, edge.weight, OL_dict[x], CL_dict[x] = FRASTAR( workSpace, all_start_loc[y], all_goal_loc[x], OL_dict[x], CL_dict[x], rtype = 'pathAndCost' )
										one_path, weight, CL_count = ASTAR( workSpace, bg, bv, rtype = 'pathAndCost' )
									path_all[bg][bv] = one_path[::-1]

								w = one_path[1]  # taking index = 1 position in the path as the next position; index = 0 is current loc

							# w is found, now check if someone (c) is at w 
							current_loc_collision_2 = 0
							for res in result_h:
								c = res[0][0]
								
								cv = path_cfree[c][-1]  # it is a location coordinate 

								if b != c and w == cv:
									current_loc_collision_2 = 1
									break
							
							if current_loc_collision_2 == 0:   # no further chance of deadlock
								break
							
							A_prime.add(b)
							b = c

							bv = path_cfree[b][-1]  # it is a location coordinate 

							bg = all_goal_loc[ current_goal[b] ]   # it is a location coordinate 



							if b == x:
								break  # cycle found

							if b in A_prime:
								A_prime.clear()
								break
						
						# while loop over
						if A_prime and (b == x):
							# rotate goal
							A_prime_list = list(A_prime)
							temp = current_goal[A_prime_list[0]]
							
							for i in range(len(A_prime_list) - 1):
								current_goal[A_prime_list[i]] = current_goal[A_prime_list[i+1]]
							
							current_goal[A_prime_list[-1]] = temp


					break

			if current_loc_collision == 0:
				path_cfree[x].append(u)
				# determine cost
				if ws_type != '3D':
					move = (u[0] - agent_curr_loc[0], u[1] - agent_curr_loc[1]) 
			
					if move in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
						individual_path_cost[x].append(1)
					else:
						individual_path_cost[x].append(1.5)
				
				elif ws_type == '3D':
					move = (u[0] - agent_curr_loc[0], u[1] - agent_curr_loc[1], u[2] - agent_curr_loc[2]) 

					if move == (0, -1, -1) or move == (0, -1, 1) or move == (0, 1, -1) or move == (0, 1, 1) or \
						move == (1, 0, -1) or move == (1, 0, 1) or move == (1, -1, 0) or move == (1, 1, 0) or \
							move == (-1, 0, -1) or move == (-1, 0, 1) or move == (-1, -1, 0) or move == (-1, 1, 0):
						individual_path_cost[x].append(1.5)
					elif move == (1, -1, -1) or move == (1, -1, 1) or move == (1, 1, -1) or move == (1, 1, 1) or \
						move == (-1, -1, -1) or move == (-1, -1, 1) or move == (-1, 1, -1) or move == (-1, 1, 1):
						individual_path_cost[x].append(2)
					else:
						individual_path_cost[x].append(1)    
            

				

		# end for each agent

	# end of main logic

	t2 = time.time()

	col_free_time = t2 - t1

	# return time and final total cost of collision free paths
	

	# compute makespan & total cost of coll-free paths:

	makespan = 0
	
	for robot in individual_path_cost:
		all_path_cost = all_path_cost + sum(individual_path_cost[robot])
		if sum(individual_path_cost[robot]) > makespan:
			makespan = sum(individual_path_cost[robot])

		# one_path = path_cfree[robot]
		# for i in range(len(one_path)-1):
		# 	move = (one_path[i+1][0] - one_path[i][0], one_path[i+1][1] - one_path[i][1]) 
		# 	if move in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
		# 		col_free_cost = col_free_cost + 1
		# 	elif move == (0, 0):
		# 		col_free_cost = col_free_cost + 0.5
		# 	else:
		# 		col_free_cost = col_free_cost + 1.5

	
	# print("Total cost (coll-free): ", col_free_cost)


	return col_free_time, path_cfree, all_path_cost, makespan


# **********************************************************************************************************************
