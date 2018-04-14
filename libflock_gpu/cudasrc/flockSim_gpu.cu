#include <iostream>

// For the CUDA runtime routines (prefixed with "cuda_")
#include <cuda_runtime.h>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <device_functions.h>

#include "nearestneighbour_gpu.cuh"


// Needed for output functions within the kernel
#include <stdio.h>

// printf() is only supported
// for devices of compute capability 2.0 and higher
#if defined(__CUDA_ARCH__) && (__CUDA_ARCH__ < 200)
   #define printf(f, ...) ((void)(f, __VA_ARGS__),0)
#endif

// For thrust routines (e.g. stl-like operators and algorithms on vectors)
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>


#include<sys/time.h>

// My own include function to generate some randomness
#include "random.cuh"

#include "flockSim_gpu.h"

#include "flock_gpu.h"







void FlockGPU::FlockingSim()
{




    // First thing is we'll generate a big old vector of random numbers for the purposes of
    // fleshing out our point data. This is much faster to do in one step than 3 seperate
    // steps.
    thrust::device_vector<float> d_Rand(NUM_POINTS*3);


    float * d_Rand_ptr = thrust::raw_pointer_cast(&d_Rand[0]);
    randFloats(d_Rand_ptr, NUM_POINTS*3);

    // We'll store the components of the 3d vectors in separate arrays.
    // This 'structure of arrays' (SoA) approach is usually more efficient than the
    // 'array of structures' (AoS) approach.  The primary reason is that structures,
    // like Float3, don't always obey the memory coalescing rules, so they are not
    // efficiently transferred to and from memory.  Another reason to prefer SoA to
    // AoS is that we don't aways want to process all members of the structure.  For
    // example, if we only need to look at first element of the structure then it
    // is wasteful to load the entire structure from memory.  With the SoA approach,
    // we can chose which elements of the structure we wish to read.
    thrust::device_vector<float> d_Px(d_Rand.begin(), d_Rand.begin()+NUM_POINTS);
    thrust::device_vector<float> d_Py(d_Rand.begin()+NUM_POINTS, d_Rand.begin()+2*NUM_POINTS);
//    thrust::device_vector<float> d_Pz(d_Rand.begin()+2*NUM_POINTS, d_Rand.end());

    // This vector will hold the grid cell occupancy (set to zero)
    thrust::device_vector<unsigned int> d_cellOcc(GRID_RESOLUTION*GRID_RESOLUTION, 0);

    // This vector will hold our hash values, one for each point
    thrust::device_vector<unsigned int> d_hash(NUM_POINTS);
    //thrust::copy(d_hash.begin(), d_hash.end(), std::ostream_iterator<unsigned int>(std::cout, " "));

    // Vector storing neighbour pnts idx
    thrust::device_vector<float> d_neighbours(NUM_POINTS,NULL_PNT);

    // Holds neighbour cells
    thrust::device_vector<unsigned int> d_neighbourCells(GRID_RESOLUTION*GRID_RESOLUTION, NULL_CELL);

    // Typecast some raw pointers to the data so we can access them with CUDA functions
    unsigned int * d_hash_ptr = thrust::raw_pointer_cast(&d_hash[0]);
    unsigned int * d_cellOcc_ptr = thrust::raw_pointer_cast(&d_cellOcc[0]);
    float * d_neighbours_ptr = thrust::raw_pointer_cast(&d_neighbours[0]);
    unsigned int * d_neighbourCells_ptr = thrust::raw_pointer_cast(&d_neighbourCells[0]);
    float * d_Px_ptr = thrust::raw_pointer_cast(&d_Px[0]);
    float * d_Py_ptr = thrust::raw_pointer_cast(&d_Py[0]);
//   float * d_Pz_ptr = thrust::raw_pointer_cast(&d_Pz[0]);

    // The number of threads per blockshould normally be determined from your hardware, but 1024
    // is pretty standard. Remember that each block will be assigned to a single SM, with it's
    // own local memory.
    unsigned int nThreads = 1024;
    unsigned int nBlocks = NUM_POINTS / nThreads + 1;


    //dim3 threadsPerBlock(8, 8);
    //dim3 numBlocks(GRID_RESOLUTION/threadsPerBlock.x, GRID_RESOLUTION/threadsPerBlock.y);

     int blockDim = 1024 / GRID_RESOLUTION + 1; // 9 threads per block
     dim3 block(GRID_RESOLUTION, GRID_RESOLUTION); // block of (X,Y) threads
     dim3 grid(1, 1); // grid 2x2 blocks

     // for nearest neighbour
     unsigned int blockN = NUM_POINTS/ (GRID_RESOLUTION*GRID_RESOLUTION*NUM_POINTS) + 1;
     dim3 block2(GRID_RESOLUTION*GRID_RESOLUTION, NUM_POINTS); // block of (X,Y) threads
     dim3 grid2(blockN, blockN); // grid 2x2 blocks


    struct timeval tim;
    double t1, t2;
    gettimeofday(&tim, NULL);
    t1=tim.tv_sec+(tim.tv_usec/1000000.0);

    // The special CUDA syntax below executes our parallel function with the specified parameters
    // using the number of blocks and threads provided.
    pointHash<<<nBlocks, nThreads>>>(d_hash_ptr, d_Px_ptr, d_Py_ptr,
                                     NUM_POINTS,
                                     GRID_RESOLUTION);

    // Make sure all threads have wrapped up before completing the timings
    cudaThreadSynchronize();

    // Now we can sort our points to ensure that points in the same grid cells occupy contiguous memory
    thrust::sort_by_key(d_hash.begin(), d_hash.end(),
                        thrust::make_zip_iterator(
                            thrust::make_tuple( d_Px.begin(), d_Py.begin())));

    // Make sure all threads have wrapped up before completing the timings
    cudaThreadSynchronize();

    // Now we can count the number of points in each grid cell
    countCellOccupancy<<<nBlocks, nThreads>>>(d_cellOcc_ptr, d_hash_ptr, d_cellOcc.size(), d_hash.size());

    // Make sure all threads have wrapped up before completing the timings
    cudaThreadSynchronize();

    neighbourhoodCells<<<grid, block>>>(d_neighbourCells_ptr,0.24,GRID_RESOLUTION, 6);

    cudaThreadSynchronize();

    // sort into order
    thrust::sort(d_neighbourCells.begin(), d_neighbourCells.end());

    emptyCellCheck<<<nBlocks,nThreads>>>(d_neighbourCells_ptr, d_cellOcc_ptr,GRID_RESOLUTION);

    cudaThreadSynchronize();

    // sort into order again
    thrust::sort(d_neighbourCells.begin(), d_neighbourCells.end());

    // Finds pnt index in neighbourhood cells
    nearestNeighbourPnts<<<grid2, block2>>>(d_neighbours_ptr, d_neighbourCells_ptr, d_hash_ptr,NUM_POINTS,GRID_RESOLUTION);

    cudaThreadSynchronize();

    // sort into order finally (PROBABLY UNNECCESERY!)
    thrust::sort(d_neighbours.begin(), d_neighbours.end());




    gettimeofday(&tim, NULL);
    t2=tim.tv_sec+(tim.tv_usec/1000000.0);
    std::cout << "Grid sorted "<<NUM_POINTS<<" points into grid of "<<GRID_RESOLUTION*GRID_RESOLUTION*GRID_RESOLUTION<<" cells in " << t2-t1 << "s\n";

    // Only dump the debugging information if we have a manageable number of points.
    if (NUM_POINTS <= 100) {
        thrust::copy(d_neighbourCells.begin(), d_neighbourCells.end(), std::ostream_iterator<unsigned int>(std::cout, " "));
        std::cout << "\n";
        thrust::copy(d_neighbours.begin(), d_neighbours.end(), std::ostream_iterator<float>(std::cout, " "));
        std::cout << "\n";
        thrust::copy(d_hash.begin(), d_hash.end(), std::ostream_iterator<unsigned int>(std::cout, " "));
        std::cout << "\n";
        thrust::copy(d_cellOcc.begin(), d_cellOcc.end(), std::ostream_iterator<unsigned int>(std::cout, " "));
    }
    //return 0;
}



