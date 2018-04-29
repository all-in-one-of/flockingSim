# GPU nad CPU Flocking
## The Project

The project in which I have chosen to parallelize for this assignment is a flocking system that uses Craig Roberts theories on flocking and steering to simulate a group of boids flocking together. 

Due to the complexity of running OpenGL on the GPU, the boids positions are exported as .bgeo files and imported into Houdini for visualization. Within the accompanying flock.hip file you can see the out come of flock simulated both on the CPU and the GPU.


## User Notes
 
Due to the restrictions wothin the kernal of fixed size arrays the number of boids for the gpu needs to be defined twice, once in the constructot and once within the header file. This is incinvenient but neccessary as the #defined version in the kernal header file must be used for the arrays within the kernals. This means that the benchmark will not automatically update and to benchmark different amounts of boids you must update the #define.

A fixed radius nearest neighbour algorithm was developed as part of this assignment and was successfully completed for both the CPU and the GPU. However due to the need to calculate the neighbour cells for every boid each frame based on their current cell from a hash table, and then to further calculate which other boids are in these cells and use these boids for flocking it made the algorithm impossibly slow and did not aid at all even with large numbers of boids.

## Parallellization

#Kernals

Alignment
Seperation
Cohesion
Avoid Boundaries
Flock
Distance
Magnitude
Update Pos
LimitVel

## GPU and CPU Comparision
All on gpu except passing device vector positions to dump geo function to export as .bgeo file for Houdini. Can be entirely on gpu using openGl.


| Function        | CPU Time          | GPU Time  |
| --------------- |:-----------------:| ---------:|
| Constructor     |                   |           |
| Update          |                   |           |
| DumpGeo         |                   |           |




# Time to simulate and output 150 Frames of Flocking

| No Boids        | CPU Time          | GPU Time    |
| --------------- |:-----------------:| -----------:|
| 100             | ~ 4.06695s        | ~ 1.91852s  |
| 200             | ~ 8.95648s        | ~ 2.56638s  |
|                 |                   |             |



## Future Improvesments
A future improvement would be to implement OpenGL on the gpu also to eliminate the need for passing data between device and host memory entirely. This would decrease computation time due to the slowness of this process.
