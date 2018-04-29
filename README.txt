# GPU nad CPU Flocking
## The Project

The project in which I have chosen to parallelize for this assignment is a flocking system that uses Craig Roberts theories on flocking and steering to simulate a group of boids flocking together. 

Due to the complexity of running OpenGL on the GPU, the boids positions are exported as .bgeo files and imported into Houdini for visualization. Within the accompanying flock.hip file you can see the out come of flock simulated both on the CPU and the GPU.

## GPU and CPU Comparision
All on gpu except passing device vector positions to dump geo function to export as .bgeo file for Houdini. Can be entirely on gpu using openGl.


| Function        | CPU Time          | GPU Time  |
| --------------- |:-----------------:| ---------:|
| Constructor     |                   |           |
| Update          |                   |           |
| DumpGeo         |                   |           |
