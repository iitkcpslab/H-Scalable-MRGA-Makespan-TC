-------------------------------------------------------------------------------------------------------------
H-Scalable-MRGA-Makespan-TC
-------------------------------------------------------------------------------------------------------------

This repository contains the implementation of the proposed approach OM+OTC as well as the baseline algorithms Base-1 and Base-2 discussed in the following paper: 

**Aakash and Indranil Saha. A Scalable Multi-Robot Goal Assignment Algorithm for Minimizing Mission Time followed by Total Movement Cost. In Artificial Intelligence (AIJ), 2025.**


### Guidelines for Experiments

To execute OM+OTC, you can choose one of the following options:
- Run ```python3 main_omotc_with_pp.py``` to execute OM+OTC with TSWAP's path planner
- Run ```python3 main_omotc_tswap_dijk_with_pp.py``` to execute OM+OTC with TSWAP's path planner and the baselines Base-1 and Base-2.


The program prompts the user to provide on-screen inputs.

omotc.py, base1.py, and base2.py contains the implementation of OM+OTC, BASE-1, and BASE-2 respectively.

Both our algorithm and the baselines use the common functionalities from the utils folder. 

The standard workspaces are provided in benchmark_maps folder and are ready-to-use. 

The benchmark_ws_translator.py file can be used to translate any new .map file into .txt file into the required format.

The 3D maps in .map file format do not need translation. 


For any query, kindly contact Aakash at aakashp@cse.iitk.ac.in
