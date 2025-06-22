# ------------------------------------- Code description -------------------------------------1

'''
        Trigger module - for executing OM+OTC + path planning (tswap's), Base-1, and Base-2  
'''

# ------------------------------------- Code description -------------------------------------2


import time
import numpy as np
from pathlib import Path

import omotc
import base1
import base2

import tswap_path_planner


results_folder_name = ""



def get_initial_locations_2D(robots, goals, workSpace):     # Fetches initial location of robots and the goals

	all_start_loc = {}  # for robots
	all_goal_loc  = {}  # for tasks

	
	# print("\n-------------------------------------------------------------------")
	# print("-----------Fetching input for robots' initial locations------------")
	# print("-------------------------------------------------------------------\n")
	

	i = 0
	
	while ( i < len(robots) ):

		a = np.random.randint(0, len(workSpace))
		b = np.random.randint(0, len(workSpace[0]))
		aa = (a, b)

		if aa in all_start_loc.values():  # two robots cannot have same start loc
			continue
		
		if aa[0] > (len(workSpace) - 1) or aa[0] < 0 or aa[1] > (len(workSpace[0]) - 1) or aa[1] < 0:
			continue  
		
		if workSpace[aa[0]][aa[1]] != 0:    # i.e. if obstacle is present, then it is not a valid cell
			continue  

		all_start_loc[robots[i]] = aa
		i = i + 1
		
	# print("Fetching of initial locations of robots complete...")
	# print("-------------------------------------------------------------------\n")



	# print("-------------------------------------------------------------------")
	# print("--------------Fetching input for Goals' locations------------------")
	# print("-------------------------------------------------------------------\n")
	
	i = 0
	while ( i < len(goals) ):

		a = np.random.randint(0, len(workSpace))
		b = np.random.randint(0, len(workSpace[0]))
		bb = (a, b)

		
		if bb in all_goal_loc.values():  # each goal location is distinct 
			continue
		
		
		if bb in all_start_loc.values():  # goal loc and start loc cannot be same 
			continue


		if bb[0] > (len(workSpace) - 1) or bb[0] < 0 or bb[1] > (len(workSpace[0]) - 1) or bb[1] < 0:
			continue  
		
		if workSpace[bb[0]][bb[1]] != 0:    # i.e. if obstacle is present, then it is not a valid cell
			continue  
		
		all_goal_loc[goals[i]] = bb
		i = i + 1
		
	# print("Fetching of locations of goals complete...")
	# print("-------------------------------------------------------------------\n")
   


	return all_start_loc, all_goal_loc



def get_initial_locations_3D(robots, goals, workSpace):     # Fetches initial location of robots and the goals

	all_start_loc = {}  # for robots
	all_goal_loc  = {}  # for tasks

	
	# print("\n-------------------------------------------------------------------")
	# print("-----------Fetching input for robots' initial locations------------")
	# print("-------------------------------------------------------------------\n")

	i = 0
	
	while ( i < len(robots) ):

		a = np.random.randint(0, workSpace.shape[0])
		b = np.random.randint(0, workSpace.shape[1])
		c = np.random.randint(0, workSpace.shape[2])
		aa = (a, b, c)

		if aa in all_start_loc.values():  # two robots cannot have same start loc
			continue
		
		if aa[0] > (workSpace.shape[0] - 1) or aa[0] < 0 or \
			aa[1] > (workSpace.shape[1] - 1) or aa[1] < 0 or \
			aa[2] > (workSpace.shape[2] - 1) or aa[2] < 0:
			continue  
		
		if workSpace[aa[0]][aa[1]][aa[2]] != 0:    # i.e. if obstacle is present, then it is not a valid cell
			continue  

		all_start_loc[robots[i]] = aa
		i = i + 1
		
	# print("Fetching of initial locations of robots complete...")
	# print("-------------------------------------------------------------------\n")



	# print("-------------------------------------------------------------------")
	# print("--------------Fetching input for Goals' locations------------------")
	# print("-------------------------------------------------------------------\n")
	
	i = 0
	while ( i < len(goals) ):

		a = np.random.randint(0, workSpace.shape[0])
		b = np.random.randint(0, workSpace.shape[1])
		c = np.random.randint(0, workSpace.shape[2])
		bb = (a, b, c)

		
		if bb in all_goal_loc.values():  # each goal loc is distinct
			continue
		

		if bb in all_start_loc.values():  # goal loc and start loc cannot be same 
			continue
		
		
		if bb[0] > (workSpace.shape[0] - 1) or bb[0] < 0 or \
			 bb[1] > (workSpace.shape[1] - 1) or bb[1] < 0 or \
			 bb[2] > (workSpace.shape[2] - 1) or bb[2] < 0:
			continue  
		
		if workSpace[bb[0]][bb[1]][bb[2]] != 0:    # i.e. if obstacle is present, then it is not a valid cell
			continue  
		
		all_goal_loc[goals[i]] = bb
		i = i + 1
		
	# print("Fetching of locations of goals complete...")
	# print("-------------------------------------------------------------------\n")
   


	return all_start_loc, all_goal_loc




def get_workspace( ws_type, ws_l, ws_b, obs_den ):

	
	if (ws_type == 'random'):  # Random placement of obstacles

		workSpace = np.zeros((ws_l, ws_b))             
			
		no_of_obs = ( obs_den * ws_l * ws_b ) // 100
				

		i = 1
		obstacles = set()
		while i <= no_of_obs:
			a = np.random.randint(0, ws_l)
			b = np.random.randint(0, ws_b)

			if (a, b) not in obstacles:
				obstacles.add((a, b))
				workSpace[a][b] = 1
				i = i + 1
			

	elif (ws_type == 'replay'):	
		workSpace = np.loadtxt( 'ws_10_4_4_20_r18.txt', dtype = int, skiprows=0 )
	
	elif (ws_type == 'warehouse'):	
		workSpace = np.loadtxt( 'benchmark_maps/benchmark5_warehouse-20-40-10-2-1.txt', dtype = int, skiprows=0 ) 
	
	elif (ws_type == 'boston'):
		workSpace = np.loadtxt( 'benchmark_maps/Boston_0_256_notrap.txt', dtype = int, skiprows=0 ) 
	
	elif (ws_type == 'paris'):
		workSpace = np.loadtxt( 'benchmark_maps/Paris_1_256_notrap.txt', dtype = int, skiprows=0 )

	elif (ws_type == 'sydney'):
		workSpace = np.loadtxt( 'benchmark_maps/Sydney_0_256_notrap.txt', dtype = int, skiprows=0 )

	elif (ws_type == 'shanghai'):
		workSpace = np.loadtxt( 'benchmark_maps/Shanghai_2_256_notrap.txt', dtype = int, skiprows=0 )

	elif (ws_type == 'mansion'):
		workSpace = np.loadtxt( 'benchmark_maps/benchmark8_ht_mansion_n.txt', dtype = int, skiprows=0 ) 
	
	elif (ws_type == 'den'):
		workSpace = np.loadtxt( 'benchmark_maps/den520d.txt', dtype = int, skiprows=0 )
	
	elif (ws_type == '3D'):    # Configuring 3D workspace using a given configuration file
		
		with open('benchmark_maps/Complex.3dmap', 'r') as f:
    
			first_line = f.readline()
			first_line_split = first_line.split()
			

			workSpace = np.zeros((int(first_line_split[1]), int(first_line_split[2]), int(first_line_split[3])), dtype=int)
			
			
			for line in f:
				split_nos = line.split()
				workSpace[int(split_nos[0])][int(split_nos[1])][int(split_nos[2])] = 1


		f.close()
		xs,ys,zs = np.where(workSpace != 0) # main line 1
		workSpace = workSpace[min(xs):max(xs)+4,min(ys):max(ys)+4,min(zs):max(zs)+4]    # main line 2
		print("Reduced 3D workspace's shape: ", workSpace.shape)





	return workSpace






def execute_set( set_param ):

	
	global results_folder_name

	ws_type = set_param['ws_type']
	no_of_robots = set_param['n_o_r']
	no_of_goals = set_param['n_o_g']
	
	loop_no = 1
	total_loops = set_param['total_loops']
	verbosity = set_param['verbosity']
	
	
	# H = Heuristic, i.e. OMOTC, i.e. our approach
	# T = TSWAP's version
	# D = Dijkstra's approach


	collect_times_taken_H = []
	collect_times_taken_H_om = []
	collect_times_taken_H_otc = []

	collect_times_taken_T = []
	collect_times_taken_T_om = []
	collect_times_taken_T_otc = []
	
	
	collect_times_taken_D = []
	collect_times_taken_D_om = []
	collect_times_taken_D_otc = []
	
	
	
	collect_nexp_makespan_H = []
	collect_nexp_makespan_T = []
	collect_nexp_makespan_D = []
	
	
	collect_no_of_exp_H = []
	collect_no_of_exp_T = []
	collect_no_of_exp_D = []

	collect_total_cost_H = []
	collect_total_cost_T = []
	collect_total_cost_D = []
	collect_om_cost_H = []

	collect_makespan_H = []
	collect_makespan_T = []
	collect_makespan_D = []

	
	collect_heur_CL = []
	collect_heur_CL_om = []

	collect_tswap_CL = []
	collect_tswap_CL_om = []

	collect_dijk_CL = []


	collect_col_free_time_H = []
	collect_col_free_cost_H = []
	collect_col_free_makespan_H = []
	
	

	log_grouped_data = open(results_folder_name + '1_grouped_data.txt', 'w')

	
	while ( loop_no <= total_loops ):
		

		workSpace = get_workspace(ws_type, set_param['ws_l'], set_param['ws_b'], set_param['obs_den'] )
		

		robots = []
		goals  = []

		
		if ws_type == 'replay':
			all_start_loc = {'r0': (6, 2), 'r1': (0, 9), 'r2': (1, 7), 'r3': (4, 1)}
			all_goal_loc = {'g0': (6, 3), 'g1': (5, 5), 'g2': (5, 9), 'g3': (5, 1)}
			


			no_of_robots = len(all_start_loc)
			no_of_goals = len(all_goal_loc)

			for i in range(no_of_robots):
				robots.append('r' + str(i))

			for j in range(no_of_goals):
				goals.append('g' + str(j))
		
		
		else:
			
		
			for i in range(no_of_robots):
				robots.append('r' + str(i))

			for j in range(no_of_goals):
				goals.append('g' + str(j))
				
		
			# Fetching start locations of all robots and goal locations of all tasks 

			if ws_type == '3D':
				all_start_loc, all_goal_loc = get_initial_locations_3D(robots, goals, workSpace)
			else:
				all_start_loc, all_goal_loc = get_initial_locations_2D(robots, goals, workSpace)


		
		set_name = str(workSpace.shape[0]) + '_' + str(no_of_robots) + '_' + str(no_of_goals) + '_' + str(set_param['obs_den']) 
		exec_id = set_name + '_r' + str(loop_no)
		
		ws_fname =   results_folder_name + 'ws_' + exec_id  + '.txt'   # this is filename in which workSpace would be saved
		log_fname = results_folder_name + 'log_' + exec_id  + '.txt'   # this is filename in which all output would be saved
		
		logf = open(log_fname, 'w')
		

		
		print("Execution ID: ", exec_id, file=logf)	
		print("-----------------------------------------------------------------------------------------\n", file=logf)
		print("Initial details: \n", file=logf)
		
		if ws_type == '3D':
			print("WorkSpace size: ", workSpace.shape[0], "*", workSpace.shape[1], "*", workSpace.shape[2], file=logf)
		else:
			print("WorkSpace size: ", workSpace.shape[0], "*", workSpace.shape[1], file=logf)

		if ws_type == 'random':
			print("Obstacle density: ", set_param['obs_den'], "%", file=logf)

		print("Number of robots: ", no_of_robots, file=logf)
		print("Number of goals: ", no_of_goals, file=logf)
		
		print("-----------------------------------------------------------------------------------------\n", file=logf)
		

				
		print("Initial location of robots : ", all_start_loc, file=logf)
		print("\nLocation of goals / tasks  : ", all_goal_loc, file=logf)
		print("\n", file=logf)
		
		logf.close()
		logf = open(log_fname, 'a')

		if ws_type == 'random':
			np.savetxt( ws_fname, workSpace, fmt='%d' )




		if verbosity > 0:
			print("\nWorkspace is ready")

	

		print("\n\n-----------------------------------------------------------------------------------------", file=logf)
		print("                                 OM+OTC begins ", file=logf)
		print("-----------------------------------------------------------------------------------------\n\n", file=logf)
		

		total_Htime = 0
		
		
		# # Invoking OM+OTC 
		# ----------------------------------------------------------------------------------------------------------------------------------------
		result_h, om_cost_h, total_cost_h, path_h, u_count_h, nexp_makespan_h, no_of_exp_h, makespan_h, heur_CL, makespan_CL, heur_t_om, heur_t_otc = omotc.heuristic ( no_of_robots, no_of_goals, workSpace, all_start_loc, all_goal_loc, ws_type )
		# ----------------------------------------------------------------------------------------------------------------------------------------


		if result_h is None or u_count_h > 0:
			if verbosity > 0:
				print("Found trapped robot/goal... retrying...\n")
			continue


		total_Htime = (heur_t_om + heur_t_otc)


		# Calling TSWAP's path planning module 
		col_free_time_h, col_free_path_h, col_free_cost_h, col_free_makespan_h = tswap_path_planner.tswap_path_planning ( result_h, path_h, workSpace, all_start_loc, all_goal_loc, ws_type )


		print("Printing results for OM+OTC case: ", file=logf)
		print("-------------------------------\n", file=logf)
		print("Time taken for OM+OTC case: ", total_Htime, "seconds", file=logf)
		print("Time taken for OM part in OM+OTC: ", heur_t_om, "seconds", file=logf)
		print("Time taken for OTC part in OM+OTC: ", heur_t_otc, "seconds", file=logf)
		print("No of robot-goal paths explored in OM+OTC case just for optimal makespan: ", nexp_makespan_h, file=logf)
		print("No of robot-goal paths explored in OM+OTC case for optimal makespan + Hungarian: ", no_of_exp_h, file=logf)
		print("Total cost for OM only: ", om_cost_h, file=logf)
		print("Total cost for OMOTC case: ", total_cost_h, file=logf)
		print("MAKESPAN-H: ", makespan_h, file=logf)
		print("Number of goals (or robots) unassigned for OM+OTC case: ", u_count_h, file=logf)

		print("COL-FREE-TIME for OM+OTC case: ", col_free_time_h, file=logf)
		print("COL-FREE-COST for OM+OTC case: ", col_free_cost_h, file=logf)

		print("CL count for OM+OTC case OM only: ", makespan_CL, file=logf)
		print("CL count for OM+OTC case OM+OTC only: ", heur_CL, file=logf)
		print("\nDetailed result of assignment: ", file=logf)
		print("-------------------------------\n", file=logf)
		print(result_h, file=logf)
		

		print("\nPrinting all paths: ", file=logf)
		print("-------------------\n", file=logf)
		
		for res in result_h:
			print("\nPath for ", res[0][0], " to ", res[0][1], ":", file=logf)
			print( path_h [res[0][0]] [res[0][1]], file=logf )  
		
		
		print("\n\n-----------------------------------------------------------------------------------------", file=logf)
		print("                                 OMOTC completed ", file=logf)
		print("-----------------------------------------------------------------------------------------\n\n", file=logf)
		

		if verbosity > 0:
			print("\nExecution of OM+OTC completed")
			print("\nStarting execution of Baseline - BASE-2 \n")



		
		print("\n\n-----------------------------------------------------------------------------------------", file=logf)
		print("                                BASE-2 (TSWAP) begins ", file=logf)
		print("-----------------------------------------------------------------------------------------\n\n", file=logf)
		
			

		total_Ttime = 0

		
		# TSWAP with FRAstar
		result_tswap, total_cost_tswap, path_tswap, u_count_tswap, makespan_nexp_tswap, total_nexp_tswap, makespan_tswap, makespan_CL_tswap, total_CL_tswap, t_om_tswap, t_otc_tswap = base2.heuristic_tswap( no_of_robots, no_of_goals, workSpace, all_start_loc, all_goal_loc, ws_type, verbosity )
		# ------------------------------------------------------------------------------------------------------------------------------


		total_Ttime = (t_om_tswap + t_otc_tswap)

		
		print("\nPrinting results for BASE-2 (TSWAP) case: ", file=logf)
		print("--------------------------------\n", file=logf)
		print("Time taken for TSWAP case: ", total_Ttime, "seconds", file=logf)
		print("Time taken for TSWAP case - OM: ", t_om_tswap, "seconds", file=logf)
		print("Time taken for TSWAP case - OTC: ", t_otc_tswap, "seconds", file=logf)
		print("No of robot-goal paths explored in TSWAP case OM only: ", makespan_nexp_tswap, file=logf)
		print("No of robot-goal paths explored in TSWAP case OM+OTC: ", total_nexp_tswap, file=logf)
		print("Total cost for TSWAP case: ", total_cost_tswap, file=logf)
		print("MAKESPAN-TSWAP: ", makespan_tswap, file=logf)
		print("Number of goals (or robots) unassigned for TSWAP case: ", u_count_tswap, file=logf)

		print("CL count for TSWAP case OM only: ", makespan_CL_tswap, file=logf)
		print("CL count for TSWAP case OM+OTC only: ", total_CL_tswap, file=logf)
		print("\nDetailed result of assignment: ", file=logf)
		print("-------------------------------\n", file=logf)
		print(result_tswap, file=logf)
		
		
		print("\n\n-----------------------------------------------------------------------------------------", file=logf)
		print("                               BASE-2 (TSWAP) completed ", file=logf)
		print("-----------------------------------------------------------------------------------------\n\n", file=logf)
		


		if verbosity > 0:
			print("\nExecution of baseline BASE-2 (TSWAP) completed")
			print("\nStarting execution of Baseline - BASE-1 \n")
		

		
		print("\n\n-----------------------------------------------------------------------------------------", file=logf)
		print("                                BASE-1 begins ", file=logf)
		print("-----------------------------------------------------------------------------------------\n\n", file=logf)
		
		
		# # Invoking function for BASE-1  case
		result_dijk, total_cost_dijk, path_dijk, u_count_dijk, no_of_exp_dijk, makespan_dijk, CL_dijk, t_om_dijk, t_otc_dijk = base1.naive( no_of_robots, no_of_goals, workSpace, all_start_loc, all_goal_loc, ws_type, verbosity )
		
		total_Dtime = (t_om_dijk + t_otc_dijk)


		print("\nPrinting results for BASE-1 case: ", file=logf)
		print("--------------------------------\n", file=logf)
		print("Time taken for BASE-1 case: ", total_Dtime, "seconds", file=logf)
		print("Time taken for BASE-1 case - OM part only: ", t_om_dijk, "seconds", file=logf)
		print("Time taken for BASE-1 case - OTC part only: ", t_otc_dijk, "seconds", file=logf)
		print("No of robot-goal paths explored in BASE-1 case both OM and OTC parts: ", no_of_exp_dijk, file=logf)
		print("Total cost for BASE-1 case: ", total_cost_dijk, file=logf)
		print("MAKESPAN-BASE-1: ", makespan_dijk, file=logf)
		print("Number of goals (or robots) unassigned for BASE-1 case: ", u_count_dijk, file=logf)
		print("CL count for BASE-1 case OM+OTC parts: ", CL_dijk, file=logf)
		print("\nDetailed result of assignment: ", file=logf)
		print("-------------------------------\n", file=logf)
		print(result_tswap, file=logf)
		
		
		print("\n\n-----------------------------------------------------------------------------------------", file=logf)
		print("                               BASE-1 completed ", file=logf)
		print("-----------------------------------------------------------------------------------------\n\n", file=logf)
		


		if verbosity > 0:
			print("\nExecution of baseline BASE-1 completed")
		



		print("-------------------", file=logf)
		print("-------------------", file=logf)
		print("    Conclusion: ", file=logf)
		print("-------------------", file=logf)
		print("-------------------\n", file=logf)
		

		
		print("\n\nOUR ALGO:--", file=logf)
		print("\nTime taken for OM+OTC case (seconds):", total_Htime, file=logf)
		print("Time taken for OM+OTC case OM: ", heur_t_om, "seconds", file=logf)
		print("Time taken for OM+OTC case OTC: ", heur_t_otc, "seconds", file=logf)
		
		print("No of robot-goal paths explored in HEUR case OM only: ", nexp_makespan_h, file=logf)
		print("No of robot-goal paths explored in HEUR case OM+OTC: ", no_of_exp_h, file=logf)
		print("Total cost for OM only: ", om_cost_h, file=logf)
		print("Total cost for OMOTC case: ", total_cost_h, file=logf)
		print("MAKESPAN-H: ", makespan_h, file=logf)
		print("COL-FREE MAKESPAN-H: ", col_free_makespan_h, file=logf)
		
		
		print("COL-FREE-TIME for OM+OTC case: ", col_free_time_h, file=logf)
		print("COL-FREE-COST for OM+OTC case: ", col_free_cost_h, file=logf)
		
		print("OM CL count: ", makespan_CL, file=logf)
		print("OM+OTC CL count: ", heur_CL, file=logf)



		

		print("\n\nBASE-2 TSWAP CASE:--", file=logf)
		print("\nTime taken for TSWAP case (seconds): ", total_Ttime, file=logf)
		print("Time taken for TSWAP case - OM: ", t_om_tswap, "seconds", file=logf)
		print("Time taken for TSWAP case - OTC: ", t_otc_tswap, "seconds", file=logf)
		
		print("No of robot-goal paths explored in TSWAP case OM only: ", makespan_nexp_tswap, file=logf)
		print("No of robot-goal paths explored in TSWAP case OM+OTC: ", total_nexp_tswap, file=logf)
		print("Total cost for TSWAP case: ", total_cost_tswap, file=logf)
		print("MAKESPAN-TSWAP: ", makespan_tswap, file=logf)

		print("OM CL count for TSWAP: ", makespan_CL_tswap, file=logf)
		print("OM+OTC CL count for TSWAP: ", total_CL_tswap, file=logf)
		print("----------------------------------------------", file=logf)



		print("\n\nBASE-1 CASE:--", file=logf)
		print("\nTime taken for BASE-1 case (seconds): ", total_Dtime, file=logf)
		print("Time taken for BASE-1 case - OM: ", t_om_dijk, "seconds", file=logf)
		print("Time taken for BASE-1 case - OTC: ", t_otc_dijk, "seconds", file=logf)
		
		print("No of robot-goal paths explored in BASE-1 case OM and OTC parts: ", no_of_exp_dijk, file=logf)
		print("Total cost for BASE-1 case: ", total_cost_dijk, file=logf)
		print("MAKESPAN-DIJK: ", makespan_dijk, file=logf)
		print("OM and OTC parts CL count for BASE-1: ", CL_dijk, file=logf)
		print("----------------------------------------------", file=logf)



		if ws_type == 'random':
			# np.savetxt( ws_fname, workSpace, fmt='%d' )
			print("\n----------------------------------------------", file=logf)
			print("Workspace saved in file ", ws_fname, file=logf)
		print("----------------------------------------------\n", file=logf)
		
		logf.close()
		
		
		# Runtime
		
		collect_times_taken_H.append( total_Htime )
		collect_times_taken_H_om.append( heur_t_om )
		collect_times_taken_H_otc.append( heur_t_otc )


		collect_times_taken_T.append( total_Ttime )
		collect_times_taken_T_om.append( t_om_tswap )
		collect_times_taken_T_otc.append( t_otc_tswap )
		

		collect_times_taken_D.append( total_Dtime )
		collect_times_taken_D_om.append( t_om_dijk )
		collect_times_taken_D_otc.append( t_otc_dijk )


		# NEXP

		collect_nexp_makespan_H.append( nexp_makespan_h ) # om
		collect_nexp_makespan_T.append( makespan_nexp_tswap ) # om
		
		collect_no_of_exp_H.append( no_of_exp_h )  # om+otc
		collect_no_of_exp_T.append( total_nexp_tswap )
		collect_no_of_exp_D.append( no_of_exp_dijk )
		
		# COST

		collect_total_cost_H.append( total_cost_h )
		collect_total_cost_T.append( total_cost_tswap )
		collect_total_cost_D.append( total_cost_dijk )
		collect_om_cost_H.append( om_cost_h )
		

		# MAKESPAN

		collect_makespan_H.append( makespan_h )
		collect_makespan_T.append( makespan_tswap )
		collect_makespan_D.append( makespan_dijk )


		# CL count 

		collect_heur_CL.append( heur_CL )         # om + otc
		collect_heur_CL_om.append( makespan_CL )  # om
		
		collect_tswap_CL.append( total_CL_tswap )
		collect_tswap_CL_om.append( makespan_CL_tswap )

		
		collect_dijk_CL.append( CL_dijk )


		# COL-FREE-TIME

		collect_col_free_time_H.append( col_free_time_h )


		# COL-FREE-COST

		collect_col_free_cost_H.append( col_free_cost_h )
		

		# COL-FREE-MAKESPAN

		collect_col_free_makespan_H.append( col_free_makespan_h )


		if verbosity > 0:
			print("\nRound ", loop_no, " completed - - - - - - - - - - - - - - - \n")
		loop_no = loop_no + 1


	# Summarizing

	time_h_mean = float( format( np.mean( collect_times_taken_H ), '.2f' ) )
	time_h_std  = float( format( np.std( collect_times_taken_H ), '.2f' ) )
	
	
	time_t_mean = float( format( np.mean( collect_times_taken_T ), '.2f' ) )
	time_t_std  = float( format( np.std( collect_times_taken_T ), '.2f' ) )

	time_d_mean = float( format( np.mean( collect_times_taken_D ), '.2f' ) )
	time_d_std  = float( format( np.std( collect_times_taken_D ), '.2f' ) )


	# Partial nexp upto optimal makespan
	
	nexp_makespan_h_mean  = float( format( np.mean( collect_nexp_makespan_H ), '.2f' ) )
	nexp_makespan_h_std   = float( format( np.std( collect_nexp_makespan_H ), '.2f' ) )

	nexp_makespan_tswap_mean  = float( format( np.mean( collect_nexp_makespan_T ), '.2f' ) )
	nexp_makespan_tswap_std   = float( format( np.std( collect_nexp_makespan_T ), '.2f' ) )


	# Full nexp for optimal makespan + min total cost
	
	exp_h_mean  = float( format( np.mean( collect_no_of_exp_H ), '.2f' ) )
	exp_h_std   = float( format( np.std( collect_no_of_exp_H ), '.2f' ) )

	exp_t_mean  = float( format( np.mean( collect_no_of_exp_T ), '.2f' ) )
	exp_t_std   = float( format( np.std( collect_no_of_exp_T ), '.2f' ) )

	exp_d_mean  = float( format( np.mean( collect_no_of_exp_D ), '.2f' ) )
	exp_d_std   = float( format( np.std( collect_no_of_exp_D ), '.2f' ) )

	
	
	# cost
	
	cost_h_mean = float( format( np.mean( collect_total_cost_H ), '.2f' ) )
	cost_h_std  = float( format( np.std( collect_total_cost_H ), '.2f' ) )
	
	cost_t_mean = float( format( np.mean( collect_total_cost_T ), '.2f' ) )
	cost_t_std  = float( format( np.std( collect_total_cost_T ), '.2f' ) )

	cost_d_mean = float( format( np.mean( collect_total_cost_D ), '.2f' ) )
	cost_d_std  = float( format( np.std( collect_total_cost_D ), '.2f' ) )

	om_cost_h_mean = float( format( np.mean( collect_om_cost_H ), '.2f' ) )
	om_cost_h_std  = float( format( np.std( collect_om_cost_H ), '.2f' ) )	
	
	
	# makespan

	makespan_h_mean = float( format( np.mean( collect_makespan_H ), '.2f' ) )
	makespan_h_std  = float( format( np.std( collect_makespan_H ), '.2f' ) )

	makespan_t_mean = float( format( np.mean( collect_makespan_T ), '.2f' ) )
	makespan_t_std  = float( format( np.std( collect_makespan_T ), '.2f' ) )

	makespan_d_mean = float( format( np.mean( collect_makespan_D ), '.2f' ) )
	makespan_d_std  = float( format( np.std( collect_makespan_D ), '.2f' ) )


	# CL count

	heur_CL_mean = float( format( np.mean( collect_heur_CL ), '.2f' ) )
	heur_CL_std  = float( format( np.std( collect_heur_CL ), '.2f' ) )
	
	tswap_CL_mean = float( format( np.mean( collect_tswap_CL ), '.2f' ) )
	tswap_CL_std  = float( format( np.std( collect_tswap_CL ), '.2f' ) )

	
	dijk_CL_mean = float( format( np.mean( collect_dijk_CL ), '.2f' ) )
	dijk_CL_std  = float( format( np.std( collect_dijk_CL ), '.2f' ) )

	
	# CL count - OM only

	heur_CL_om_mean = float( format( np.mean( collect_heur_CL_om ), '.2f' ) )
	heur_CL_om_std  = float( format( np.std( collect_heur_CL_om ), '.2f' ) )
	
	tswap_CL_om_mean = float( format( np.mean( collect_tswap_CL_om ), '.2f' ) )
	tswap_CL_om_std  = float( format( np.std( collect_tswap_CL_om ), '.2f' ) )




	netResult_filename = results_folder_name + 'netResult.txt'
	logf_netResult = open(netResult_filename, 'a')
	print( "----------------------------------------------------------", file=logf_netResult )
	print( "Set : ", set_name, file=logf_netResult )
	print( "----------------------------------------------------------", file=logf_netResult )
	
	print("\nRUNTIME:", file=logf_netResult)	

	
	print( "\nMean runtime for HEUR case (in seconds): ", time_h_mean, file=logf_netResult )
	print( "Stdv runtime for HEUR case (in seconds): ", time_h_std, file=logf_netResult )
	
	
	print( "- - - - - - - ", file=logf_netResult )

	
	print( "\nMean runtime for BASE-2 TSWAP case (in seconds): ", time_t_mean, file=logf_netResult )
	print( "Stdv runtime for BASE-2 TSWAP case (in seconds): ", time_t_std, file=logf_netResult )

	print( "- - - - - - - ", file=logf_netResult )

	
	
	print( "\nMean runtime for HEUR case TO GET COLL-FREE PATHS (in seconds): ", float( format( np.mean( collect_col_free_time_H ), '.2f' ) ), file=logf_netResult )
	print( "Stdv runtime for HEUR case TO GET COLL-FREE PATHS (in seconds): ", float( format( np.std( collect_col_free_time_H ), '.2f' ) ), file=logf_netResult )
	
	print( "- - - - - - - ", file=logf_netResult )

	print( "\nMean runtime for BASE-1 DIJK case (in seconds): ", time_d_mean, file=logf_netResult )
	print( "Stdv runtime for BASE-1 DIJK case (in seconds): ", time_d_std, file=logf_netResult )
	
	print( "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - ", file=logf_netResult )

	print("\n\nNEXP:", file=logf_netResult)	

	print( "\nMean nexp for HEUR case OM: ", nexp_makespan_h_mean, file=logf_netResult )
	print( "Stdv nexp for HEUR case OM: ", nexp_makespan_h_std, file=logf_netResult )
		
	print( "\nMean nexp for HEUR case OM-OTC: ", exp_h_mean, file=logf_netResult )
	print( "Stdv nexp for HEUR case OM-OTC: ", exp_h_std, file=logf_netResult )

	print( "- - - - - - - ", file=logf_netResult )

	print( "\nMean nexp for TSWAP case OM : ", nexp_makespan_tswap_mean, file=logf_netResult )
	print( "Stdv nexp for TSWAP case OM: ", nexp_makespan_tswap_std, file=logf_netResult )

	
	print( "\nMean nexp for BASE-2 TSWAP case OM-OTC : ", exp_t_mean, file=logf_netResult )
	print( "Stdv nexp for BASE-2 TSWAP case OM-OTC: ", exp_t_std, file=logf_netResult )

	print( "- - - - - - - ", file=logf_netResult )

	print( "\nMean nexp for BASE-1 DIJK case OM-OTC : ", exp_d_mean, file=logf_netResult )
	print( "Stdv nexp for BASE-1 DIJK case OM-OTC: ", exp_d_std, file=logf_netResult )

	print( "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - ", file=logf_netResult )
	
	print("\n\nMAKESPAN:", file=logf_netResult)	

	print( "\n\nMean MAKESPAN for HEUR case: ", makespan_h_mean, file=logf_netResult )
	print( "Stdv MAKESPAN for HEUR case: ", makespan_h_std, file=logf_netResult )
	
	print( "- - - - - - - ", file=logf_netResult )

	print( "\nMean MAKESPAN for TSWAP case: ", makespan_t_mean, file=logf_netResult )
	print( "Stdv MAKESPAN for TSWAP case: ", makespan_t_std, file=logf_netResult )
	
	print( "- - - - - - - ", file=logf_netResult )

	print( "- - - - - - - ", file=logf_netResult )

	print( "\nMean MAKESPAN for HEUR case TO GET COLL-FREE PATHS (in seconds): ", float( format( np.mean( collect_col_free_makespan_H ), '.2f' ) ), file=logf_netResult )
	print( "Stdv MAKESPAN for HEUR case TO GET COLL-FREE PATHS (in seconds): ", float( format( np.std( collect_col_free_makespan_H ), '.2f' ) ), file=logf_netResult )
	
	print( "- - - - - - - ", file=logf_netResult )


	print( "\nMean MAKESPAN for base-1 dijk case: ", makespan_d_mean, file=logf_netResult )
	print( "Stdv MAKESPAN for base-1 dijk case: ", makespan_d_std, file=logf_netResult )
		
	print( "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - ", file=logf_netResult )

	print("\nCOST:", file=logf_netResult)	

	print( "\n\nMean cost for OM: ", om_cost_h_mean, file=logf_netResult )
	print( "Stdv cost for OM: ", om_cost_h_std, file=logf_netResult )
	
	print( "\n\nMean cost for OMOTC: ", cost_h_mean, file=logf_netResult )
	print( "Stdv cost for OMOTC: ", cost_h_std, file=logf_netResult )
	
	print( "- - - - - - - ", file=logf_netResult )
	
	print( "\nMean cost for TSWAP case: ", cost_t_mean, file=logf_netResult )
	print( "Stdv cost for TSWAP case: ", cost_t_std, file=logf_netResult )

	print( "- - - - - - - ", file=logf_netResult )

	print( "\nMean cost for HEUR case TO GET COLL-FREE PATHS (in seconds): ", float( format( np.mean( collect_col_free_cost_H ), '.2f' ) ), file=logf_netResult )
	print( "Stdv cost for HEUR case TO GET COLL-FREE PATHS (in seconds): ", float( format( np.std( collect_col_free_cost_H ), '.2f' ) ), file=logf_netResult )
	
	print( "- - - - - - - ", file=logf_netResult )
	
	print( "\nMean cost for base-1 dijk case: ", cost_d_mean, file=logf_netResult )
	print( "Stdv cost for base-1 dijk case: ", cost_d_std, file=logf_netResult )

	
	
	print( "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - ", file=logf_netResult )


	print("\n\nCL COUNT:", file=logf_netResult)	

	print( "\nMean CL count for HEUR case for OM: ", heur_CL_om_mean, file=logf_netResult )
	print( "Stdv CL count for HEUR case for OM: ", heur_CL_om_std, file=logf_netResult )
	
	
	print( "\nMean CL count for HEUR case for OM-OTC: ", heur_CL_mean, file=logf_netResult )
	print( "Stdv CL count for HEUR case for OM-OTC: ", heur_CL_std, file=logf_netResult )
	
	print( "- - - - - - - ", file=logf_netResult )

	print( "\nMean CL count for TSWAP case OM: ", tswap_CL_om_mean, file=logf_netResult )
	print( "Stdv CL count for TSWAP case OM: ", tswap_CL_om_std, file=logf_netResult )

	print( "\nMean CL count for TSWAP case OM-OTC: ", tswap_CL_mean, file=logf_netResult )
	print( "Stdv CL count for TSWAP case OM-OTC: ", tswap_CL_std, file=logf_netResult )

	print( "- - - - - - - ", file=logf_netResult )

	print( "\nMean CL count for base-1 dijk case OM-OTC: ", dijk_CL_mean, file=logf_netResult )
	print( "Stdv CL count for base-1 dijk case OM-OTC: ", dijk_CL_std, file=logf_netResult )


	print( "----------------------------------------------------------\n\n", file=logf_netResult )
	logf_netResult.close()


	# Printing individual datum in a single file

	print("\n RUNTIMES \n", file=log_grouped_data)

	print("Heur OM runtimes - TAKE IT:\n", file=log_grouped_data)

	for tt_h in collect_times_taken_H_om:
		print(tt_h, file=log_grouped_data)

	
	print("\n\nHeur OTC runtimes - TAKE IT:\n", file=log_grouped_data)

	for tt_h in collect_times_taken_H_otc:
		print(tt_h, file=log_grouped_data)


	print("\n\nHeur OM+OTC runtimes - TAKE IT:\n", file=log_grouped_data)

	for tt_h in collect_times_taken_H:
		print(tt_h, file=log_grouped_data)


	print("\n\nHeur OM+OTC runtimes TO GET COLLISON-FREE PATHS - TAKE IT:\n", file=log_grouped_data)

	for tt_h in collect_col_free_time_H:
		print(tt_h, file=log_grouped_data)




	print("\n\nBase-2 TSWAP OM runtimes:\n", file=log_grouped_data)
	
	for tt_a in collect_times_taken_T_om:
		print(tt_a, file=log_grouped_data)


	print("\nBase-2 TSWAP OTC runtimes:\n", file=log_grouped_data)
	
	for tt_a in collect_times_taken_T_otc:
		print(tt_a, file=log_grouped_data)


	print("\nBase-2 TSWAP OM+OTC runtimes - TAKE IT:\n", file=log_grouped_data)
	
	for tt_a in collect_times_taken_T:
		print(tt_a, file=log_grouped_data)


	# BASE-1 
	print("\nBase-1  OM runtimes:\n", file=log_grouped_data)
	
	for tt_a in collect_times_taken_D_om:
		print(tt_a, file=log_grouped_data)


	print("\nBase-1  OTC runtimes:\n", file=log_grouped_data)
	
	for tt_a in collect_times_taken_D_otc:
		print(tt_a, file=log_grouped_data)


	print("\nBase-1  OM+OTC runtimes - TAKE IT:\n", file=log_grouped_data)
	
	for tt_a in collect_times_taken_D:
		print(tt_a, file=log_grouped_data)



	print("\n - - - - - - - - - -- - - - - -- - - -- - \n", file=log_grouped_data)

	
	print("\n NEXP \n", file=log_grouped_data)


	print("\nHeur OM nexp:\n", file=log_grouped_data)
	
	for tnexp_h in collect_nexp_makespan_H:
		print(tnexp_h, file=log_grouped_data)

	
	print("\nHeur OM+OTC nexp - TAKE IT:\n", file=log_grouped_data)
	
	for texp_h in collect_no_of_exp_H:
		print(texp_h, file=log_grouped_data)


	print("\nBase-2 TSWAP OM nexp:\n", file=log_grouped_data)
	
	for tnexp_t in collect_nexp_makespan_T:
		print(tnexp_t, file=log_grouped_data)


	print("\nBase-2 TSWAP OM+OTC nexp - TAKE IT:\n", file=log_grouped_data)
	
	for texp_t in collect_no_of_exp_T:
		print(texp_t, file=log_grouped_data)


	print("\nBase-1 DIJK OM+OTC nexp - TAKE IT:\n", file=log_grouped_data)
	
	for texp_t in collect_no_of_exp_D:
		print(texp_t, file=log_grouped_data)


	print("\n - - - - - - - - - -- - - - - -- - - -- - \n", file=log_grouped_data)
	
	print("\n CL count \n", file=log_grouped_data)

	print("\nHeur CL count - OM:\n", file=log_grouped_data)
	
	for clc in collect_heur_CL_om:
		print(clc, file=log_grouped_data)


	
	print("\nHeur CL count - OM+OTC - TAKE IT:\n", file=log_grouped_data)
	
	for clc in collect_heur_CL:
		print(clc, file=log_grouped_data)

	

	print("\nBase-2 TSWAP CL count - OM:\n", file=log_grouped_data)
	
	for clc in collect_tswap_CL_om:
		print(clc, file=log_grouped_data)

	
	print("\nBase-2 TSWAP CL count - OM+OTC - TAKE IT:\n", file=log_grouped_data)
	
	for clc in collect_tswap_CL:
		print(clc, file=log_grouped_data)

	print("\nBase-1 DIJK CL count - OM+OTC - TAKE IT:\n", file=log_grouped_data)
	
	for clc in collect_dijk_CL:
		print(clc, file=log_grouped_data)



	print("\n - - - - - - - - - -- - - - - -- - - -- - \n", file=log_grouped_data)

	print("\n MAKESPAN \n", file=log_grouped_data)


	print("\nHeur MAKESPAN:\n", file=log_grouped_data)
	
	for tmakespan_h in collect_makespan_H:
		print(tmakespan_h, file=log_grouped_data)

	print("\nBase-2 TSWAP MAKESPAN:\n", file=log_grouped_data)
	
	for tmakespan_a in collect_makespan_T:
		print(tmakespan_a, file=log_grouped_data)


	print("\n\nHeur OM+OTC MAKESPAN for COLLISON-FREE PATHS - TAKE IT:\n", file=log_grouped_data)

	for tt_h in collect_col_free_makespan_H:
		print(tt_h, file=log_grouped_data)

	print("\nBase-1 MAKESPAN:\n", file=log_grouped_data)
	
	for tmakespan_a in collect_makespan_D:
		print(tmakespan_a, file=log_grouped_data)


	print("\n - - - - - - - - - -- - - - - -- - - -- - \n", file=log_grouped_data)

	print("\n COST \n", file=log_grouped_data)


	print("\nOM cost:\n", file=log_grouped_data)
	
	for tcost_om in collect_om_cost_H:
		print(tcost_om, file=log_grouped_data)


	print("\nOMOTC cost:\n", file=log_grouped_data)
	
	for tcost_h in collect_total_cost_H:
		print(tcost_h, file=log_grouped_data)

	print("\nBase-2 TSWAP cost:\n", file=log_grouped_data)
	
	for tcost_a in collect_total_cost_T:
		print(tcost_a, file=log_grouped_data)

	
	print("\n\nHeur OM+OTC cost for COLLISON-FREE PATHS - TAKE IT:\n", file=log_grouped_data)

	for tt_h in collect_col_free_cost_H:
		print(tt_h, file=log_grouped_data)


	print("\nBase-1 cost:\n", file=log_grouped_data)
	
	for tcost_a in collect_total_cost_D:
		print(tcost_a, file=log_grouped_data)

	

	print("\n--------------E-N-D-------------------:", file=log_grouped_data)

	log_grouped_data.close()




	return_time_h = ( time_h_mean, time_h_std )
	return_time_t = ( time_t_mean, time_t_std )

	return_exp_h = ( exp_h_mean, exp_h_std )
	return_exp_t = ( exp_t_mean, exp_t_std )


	return return_time_h, return_time_t, return_exp_h, return_exp_t


	
	

def main():

	global results_folder_name

	
	settings = {}
	

	workspace_length = 0
	workspace_width = 0
	obs_density = 0
	n_o_r = 0
	n_o_g = 0


	print("\n\n________________________________________________\n")
	print("\t\tW E L C O M E")
	print("________________________________________________\n")


	while True:
		
		print("\n\n________________________________________________\n")
		print("Following are the supported types of workspaces:\n")

		print("1. Random workspace")
		print("2. Standard workspace")
		print("3. Replay past execution")
		
		print("________________________________________________\n")

		try:
			choice = int(input("\nPlease select the workspace type or press 9 to exit: "))
		except ValueError:
			print("\nThat's not a number! Please retry...")
			continue

		if choice == 9:
			break
		
		
		
		if choice == 1:
			ws_type = 'random'

			print("\n\nPlease enter the following details --")
			workspace_length = int(input("\n\tLength of workspace: "))
			workspace_width  = int(input("\n\tWidth of workspace: "))

			obs_density  = int(input("\n\tObstacle density: "))


		elif choice == 2:

			print(" \nThese are standard workspaces supported:\n")

			print("1. Boston")
			print("2. Paris")
			print("3. Sydney")
			print("4. Shanghai")
			print("5. Warehouse")
			print("6. Mansion")
			print("7. Den")
			print("8. 3D Warframe")
			

			try:
				choice_b = int(input("\nPlease select the standard workspace or press 9 to exit: "))
			except ValueError:
				print("\nThat's not a number! Please retry...")
				continue

			if choice_b == 9:
				break


			if choice_b == 1:
				ws_type = 'boston'

			elif choice_b == 2:
				ws_type = 'paris'

			elif choice_b == 3:
				ws_type = 'sydney'
			
			elif choice_b == 4:
				ws_type = 'shanghai'

			elif choice_b == 5:
				ws_type = 'warehouse'

			elif choice_b == 6:
				ws_type = 'mansion'

			elif choice_b == 7:
				ws_type = 'den'

			elif choice_b == 8:
				ws_type = '3D'

			# elif choice_b == 7:
			# 	ws_type = 'replay'


			else:
				print("\n\nInvalid choice, please retry...")
				continue

			print("\nPlease enter the following details --")
		
		elif choice == 3:
			ws_type = 'replay'
		
		else:
			print("\n\nInvalid choice, please retry...")
			continue


		if choice != 3:
			n_o_r = int(input("\n\tNumber of robots: "))
			n_o_g = int(input("\n\tNumber of goals: "))


		total_loops = int(input("\n\tNumber of rounds for which experiment should run: "))
		results_folder_name = input("\n\tRelative folder name to store result files: ")
		results_folder_name = results_folder_name + '/'
		Path(results_folder_name).mkdir(exist_ok=True)
		# verbosity = int(input("\n\tPlease select the verbosity level(0, 1, or 2): "))
		verbosity = 2
		print("\n-------------------------------------------------------------\n")



		settings[ 'main_set'  ] = { 'ws_l': workspace_length, 'ws_b': workspace_width, 'n_o_r': n_o_r, 'n_o_g': n_o_g, 'obs_den': obs_density, 'total_loops': total_loops,'ws_type': ws_type, 'verbosity': verbosity }	


		print("Thank you for the inputs, execution has started...\n")
		print("-------------------------------------------------------------\n")

		execute_set( settings['main_set'] )

		print("-------------------------------------------------------------\n")
		print("Execution for given inputs completed...Restarting...\n")
		print("-------------------------------------------------------------\n")
	

	print("\n_______________________________")
	print("\nExecution completed, thank you!")
	print("_______________________________\n\n")



if __name__ == '__main__':
    main()

