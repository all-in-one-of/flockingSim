#include <iostream>

// For the CUDA runtime routines (prefixed with "cuda_")
#include <cuda_runtime.h>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <device_functions.h>


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

#include "flock_gpu.h"


/// The number of points to generate within 0,1
#define NUM_POINTS 50

/// The resolution of our grid (dependent on the radius of influence of each point)
#define GRID_RESOLUTION 4


/// The null hash indicates the point isn't in the grid (this shouldn't happen if your extents are correctly chosen)
#define NULL_HASH UINT_MAX

/**
  * Find the cell hash of each point. The hash is returned as the mapping of a point index to a cell.
  * If the point isn't inside any cell, it is set to NULL_HASH. This may have repercussions later in
  * the code.
  * \param Px The array of x values
  * \param Py The array of y values
  * \param Pz the array of z values
  * \param hash The array of hash output
  * \param N The number of points (dimensions of Px,Py,Pz and hash)
  * \param res The resolution of our grid.
  */
__global__ void pointHash(unsigned int *hash,
                          const float *Px,
                          const float *Py,
                          const float *Pz,
                          const unsigned int N,
                          const unsigned int res) {
    // Compute the index of this thread: i.e. the point we are testing
    uint idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < N) {
        // Note that finding the grid coordinates are much simpler if the grid is over the range [0,1] in
        // each dimension and the points are also in the same space.
        int gridPos[3];
        gridPos[0] = floor(Px[idx] * res);
        gridPos[1] = floor(Py[idx] * res);
        gridPos[2] = floor(Pz[idx] * res);

        // Test to see if all of the points are inside the grid
        bool isInside = true;
        unsigned int i;
        for (i=0; i<3; ++i)
            if ((gridPos[i] < 0) || (gridPos[i] > res)) {
                isInside = false;
            }

        // Write out the hash value if the point is within range [0,1], else write NULL_HASH
        if (isInside) {
            hash[idx] = gridPos[0] * res * res + gridPos[1] * res + gridPos[2];
        } else {
            hash[idx] = NULL_HASH;
        }
        // Uncomment the lines below for debugging. Not recommended for 4mil points!
        //printf("pointHash<<<%d>>>: P=[%f,%f,%f] gridPos=[%d,%d,%d] hash=%d\n",
        //       idx, Px[idx], Py[idx], Pz[idx],
        //       gridPos[0], gridPos[1], gridPos[2], hash[idx]);
    }
}

/**
  * Compute the grid cell occupancy from the input vector of grid hash values. Note that the hash[]
  * vector doesn't need to be presorted, but performance will probably improve if the memory is
  * contiguous.
  * \param cellOcc A vector, size GRID_RES^3, which will contain the occupancy of each cell
  * \param hash A vector, size NUM_POINTS, which contains the hash of the grid cell of this point
  * \param nCells The size of the cellOcc vector (GRID_RES^3)
  * \param nPoints The number of points (size of hash)
  */
__global__ void countCellOccupancy(unsigned int *cellOcc,
                                   unsigned int *hash,
                                   unsigned int nCells,
                                   unsigned int nPoints) {
    // Compute the index of this thread: i.e. the point we are testing
    uint idx = blockIdx.x * blockDim.x + threadIdx.x;

    // Perform a sanity check and then atomic add to the occupancy count of the relevant cell
    if ((idx < nPoints) && (hash[idx] < nCells)) {
        atomicAdd(&(cellOcc[hash[idx]]), 1);
    }
}


__device__ float distancePoints(float pntX,
                          float pntY,
                          unsigned int N,
                          float neighbourPntx,
                          float neighbourPnty)
{

    float distance = 0;

    distance = sqrt((pntX-neighbourPntx)*(pntX-neighbourPntx)+(pntY - neighbourPnty)*(pntY - neighbourPnty));


    return distance;
}



// EXTEND TO 3D
// find cells surrounding current particles cells
__global__ void neighbouringCells(float *neighbourhoodX,
                                  float *neighbourhoodY,
                                  unsigned int *cellOcc,
                                  unsigned int *hash,
                                  const float *Px,
                                  const float *Py,
                                  const float *Pz,
                                 float neighbourhoodDist,
                                const unsigned int N,
                                const unsigned int res,
                                unsigned int cell
                                )
{

    // Compute the index of this thread: i.e. the point we are testing
    uint idx = blockIdx.x * blockDim.x + threadIdx.x;

    // Compute the index of this thread: i.e. the point we are testing
    uint idy = blockIdx.y * blockDim.y + threadIdx.y;


    // the number of cells in each direction to check
    int bucketRadius = ceil(neighbourhoodDist/(1.0/float(res)));;

    //printf("idx: %d, idy %d \n", idx, idy);

    // Find surrounding cells
    int y = floor(float(cell/res));
    int x = cell -(y*res);

    int count = 0;

    int neighbourCells[GRID_RESOLUTION*GRID_RESOLUTION];


    // finds neighbours of non corner cell (2d for now)
    if(x>0 &&y>0&& x<res-1&&y<res-1)
    {
        //not a border element.

        for( int i = x - bucketRadius; i <= x + bucketRadius; ++i ){
            for( int j = y - bucketRadius; j <= y + bucketRadius; ++j ){
                if(i>=0 && j>=0 && i<=res-1 && j<= res-1)
                {
                    if((j*res + i) != cell || (j*res + i) ==0 )
                    {
                        neighbourCells[count] = (j*res) + i;
                        //neighbourhood[count]  = neighbourCells[count];

                        //neighbourhood[idx] = idy;

                        count ++;

                    }
                }

            }
        }
    }



    int count2;

    // Remove empty cells
    // go through cells
    for(int i = 0; i < count; i++)
    {
        // if cell not empty
        if(cellOcc[neighbourCells[i]] == 0)
        {
            //add points to neighbourhood

            printf("deleting cell %d \n", neighbourCells[i]);
            neighbourCells[i] = -1;

            count2++;


        }


    }

    float neighbourCellPntsX[NUM_POINTS];
    float neighbourCellPntsY[NUM_POINTS];


    // order neighbours cells and iterate with while loop

    // find points in cells
    for(int i = 0; i < N; i++)
    {
        for(int j = 0; j<=count; j++)
        {

            if(hash[i] == neighbourCells[j])
            {



                  neighbourCellPntsX[i]=Px[i];
                  neighbourCellPntsY[i]=Py[i];
                  //neighbourhood[(3*i)+2]=Pz[i];

                  // exit for loop

            }
        }
    }

    float d = 0;

    d = distancePoints(neighbourhoodX[0],neighbourhoodY[0],N,0.0,0.1);

    // eliminate points not in neighbourhood by distance

    for(int i = 0; i<N; i++)
    {
        if(distancePoints(neighbourCellPntsX[i],neighbourCellPntsY[i],N,0.0,0.0) <= neighbourhoodDist)
        {
            // add point to neighbours
            neighbourhoodX[i]=neighbourCellPntsX[i];
            neighbourhoodY[i]=neighbourCellPntsY[i];


        }
    }





//        int i = (x-bucketRadius) + idx;
//        int j = (y-bucketRadius) + idy;

//        if(i<=x + bucketRadius && j<=y + bucketRadius)
//        {
//            if((j*res + i) != cell)
//            {
//                neighbourhood[idy + idx] = (j*res) + i;

//            }
//        }


}





void FlockGPU::nearestNeighbour()
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
    thrust::device_vector<float> d_Pz(d_Rand.begin()+2*NUM_POINTS, d_Rand.end());

    // This vector will hold the grid cell occupancy (set to zero)
    thrust::device_vector<unsigned int> d_cellOcc(GRID_RESOLUTION*GRID_RESOLUTION*GRID_RESOLUTION, 0);

    // This vector will hold our hash values, one for each point
    thrust::device_vector<unsigned int> d_hash(NUM_POINTS);
    //thrust::copy(d_hash.begin(), d_hash.end(), std::ostream_iterator<unsigned int>(std::cout, " "));

    // Vector storing neighbours
    thrust::device_vector<float> d_neighbours_x(NUM_POINTS);
    thrust::device_vector<float> d_neighbours_y(NUM_POINTS);

    // Typecast some raw pointers to the data so we can access them with CUDA functions
    unsigned int * d_hash_ptr = thrust::raw_pointer_cast(&d_hash[0]);
    unsigned int * d_cellOcc_ptr = thrust::raw_pointer_cast(&d_cellOcc[0]);
    float * d_neighboursX_ptr = thrust::raw_pointer_cast(&d_neighbours_x[0]);
    float * d_neighboursY_ptr = thrust::raw_pointer_cast(&d_neighbours_y[0]);
    float * d_Px_ptr = thrust::raw_pointer_cast(&d_Px[0]);
    float * d_Py_ptr = thrust::raw_pointer_cast(&d_Py[0]);
    float * d_Pz_ptr = thrust::raw_pointer_cast(&d_Pz[0]);

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


    struct timeval tim;
    double t1, t2;
    gettimeofday(&tim, NULL);
    t1=tim.tv_sec+(tim.tv_usec/1000000.0);

    // The special CUDA syntax below executes our parallel function with the specified parameters
    // using the number of blocks and threads provided.
    pointHash<<<nBlocks, nThreads>>>(d_hash_ptr, d_Px_ptr, d_Py_ptr, d_Pz_ptr,
                                     NUM_POINTS,
                                     GRID_RESOLUTION);

    // Make sure all threads have wrapped up before completing the timings
    cudaThreadSynchronize();

    // Now we can sort our points to ensure that points in the same grid cells occupy contiguous memory
    thrust::sort_by_key(d_hash.begin(), d_hash.end(),
                        thrust::make_zip_iterator(
                            thrust::make_tuple( d_Px.begin(), d_Py.begin(), d_Pz.begin())));

    // Make sure all threads have wrapped up before completing the timings
    cudaThreadSynchronize();

    // Now we can count the number of points in each grid cell
    countCellOccupancy<<<nBlocks, nThreads>>>(d_cellOcc_ptr, d_hash_ptr, d_cellOcc.size(), d_hash.size());

    // Make sure all threads have wrapped up before completing the timings
    cudaThreadSynchronize();

    // Testing nearest neighbourhood
    neighbouringCells<<<grid, block>>>(d_neighboursX_ptr,d_neighboursY_ptr,d_cellOcc_ptr, d_hash_ptr, d_Px_ptr, d_Py_ptr, d_Pz_ptr,0.24,NUM_POINTS,GRID_RESOLUTION,6);

    // Make sure all threads have wrapped up before completing the timings
    cudaThreadSynchronize();

    gettimeofday(&tim, NULL);
    t2=tim.tv_sec+(tim.tv_usec/1000000.0);
    std::cout << "Grid sorted "<<NUM_POINTS<<" points into grid of "<<GRID_RESOLUTION*GRID_RESOLUTION*GRID_RESOLUTION<<" cells in " << t2-t1 << "s\n";

    // Only dump the debugging information if we have a manageable number of points.
    if (NUM_POINTS <= 100) {
        thrust::copy(d_neighbours_x.begin(), d_neighbours_x.end(), std::ostream_iterator<float>(std::cout, " "));
        std::cout << "\n";
        thrust::copy(d_Px.begin(), d_Px.end(), std::ostream_iterator<float>(std::cout, " "));
        std::cout << "\n";
        thrust::copy(d_hash.begin(), d_hash.end(), std::ostream_iterator<unsigned int>(std::cout, " "));
        std::cout << "\n";
        thrust::copy(d_cellOcc.begin(), d_cellOcc.end(), std::ostream_iterator<unsigned int>(std::cout, " "));
    }
    //return 0;
}



