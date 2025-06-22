import numpy as np
from numpy.core.fromnumeric import shape


workSpace = np.loadtxt( 'Boston_0_256.map', dtype = str, skiprows=4 )   # <<<<<<<-----------------Change here 1 --


workSpace_2 = np.zeros((len(workSpace), len(workSpace[0])), dtype=int)

for i in range(len(workSpace)):
    for j in range(len(workSpace[0])):
        if workSpace[i][j] == '@' or workSpace[i][j] == 'T':         # <<<<--- Change here 2 ---change character here as per the map --
            workSpace_2[i][j] = 1
        if workSpace[i][j] == '.':
            workSpace_2[i][j] = 0


np.savetxt( 'benchmark6_Boston_0_256.txt', workSpace_2, fmt='%d' )   # <<<<<<<------- Change here 3 --
print("resulting workspace shape: ", workSpace_2.shape)

