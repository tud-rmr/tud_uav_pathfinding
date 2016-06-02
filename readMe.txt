This programm is tested for V-REP in combination spyder.

Mapdata:
The file "testroom111.npy" contains the mapdata. Its located in the Mapdata-folder. Its loaded automatically, if the mapgen()-function is called for "testroom111.npy"

Simulation scene:
"testroom111.ttt"

Programm code:
The code is located in "UAV_main.py" it contains all steps from mapgeneration->path finding->path smoothing. For each step functions from other files are called.

Programm execution:
To run the pathfinding programm first you need to start the simulation in V-REP, then its necessary to run the whole code in "UAV_main.py".