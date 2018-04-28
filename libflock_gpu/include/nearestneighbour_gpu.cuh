#ifndef NEARESTNEIGHBOUR_H
#define NEARESTNEIGHBOUR_H

#include <cuda_runtime.h>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <device_functions.h>


#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>

#include <iostream>
#include <stdio.h>

// printf() is only supported
// for devices of compute capability 2.0 and higher
#if defined(__CUDA_ARCH__) && (__CUDA_ARCH__ < 200)
   #define printf(f, ...) ((void)(f, __VA_ARGS__),0)
#endif



/// The number of points to generate within 0,1
#define NUM_BOIDS 100

/// The resolution of our grid (dependent on the radius of influence of each point)
#define GRID_RESOLUTION 4


/// Used to define a point not in the neighbourhood
#define NULL_PNT UINT_MAX

/// Used to define a non neighbourhood cell
#define NULL_CELL UINT_MAX

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
                          const unsigned int N,
                          const unsigned int res) {
    // Compute the index of this thread: i.e. the point we are testing
    uint idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < N) {
        // Note that finding the grid coordinates are much simpler if the grid is over the range [0,1] in
        // each dimension and the points are also in the same space.
        int gridPos[2];
        gridPos[0] = floor(Px[idx] * res);
        gridPos[1] = floor(Py[idx] * res);
        //gridPos[2] = floor(Pz[idx] * res);

        // Test to see if all of the points are inside the grid
        bool isInside = true;
        unsigned int i;
        for (i=0; i<2; ++i)
            if ((gridPos[i] < 0) || (gridPos[i] > res)) {
                isInside = false;
            }

        // Write out the hash value if the point is within range [0,1], else write NULL_HASH
        if (isInside) {
            hash[idx] = gridPos[0] + (gridPos[1] * res);
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
  * \param hash A vector, size NUM_BOIDS, which contains the hash of the grid cell of this point
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



// DOESNT WORK WITH MORE THEN ONE CELL RADIUS, PROBLEM WITH THREADS

__global__ void neighbourhoodCells(unsigned int *neighbourCells,
                                   float neighbourhoodDist,
                                  const unsigned int res,
                                  unsigned int cell)
{

    // Compute the index of this thread: i.e. the point we are testing
    //uint idx = blockIdx.x * blockDim.x + threadIdx.x;

    // Compute the index of this thread: i.e. the point we are testing
   // uint idy = blockIdx.y * blockDim.y + threadIdx.y;




    // the number of cells in each direction to check
    int bucketRadius = ceil(neighbourhoodDist/(1.0/float(res)));;

    // Find surrounding cells
    int y = floor(float(cell/res));
    int x = cell -(y*res);



//    int count = 0;

    // add neighbour cells and current cell to array
//    if(idx <= 2 * bucketRadius && idy <= 2 * bucketRadius)
//    {
//        //printf("idx: %d, idy %d \n", idx, idy);

//        int i = x - bucketRadius + idx;
//        int j = y - bucketRadius + idy;

//        //printf("%d i, %d j \n", i , j);

//        //printf("%d idx, %d idy \n", idx , idy);

//        if(i>=0 && j>=0 && i<=res-1 && j<= res-1)
//        {
//            neighbourCells[(j*res) + i] = (j*res) + i;
//            //printf(" %d neighbour cell added \n",neighbourCells[(j*res) + i]);

//            //count ++;
//        }

//    }


    int count = 0;

    //int neighbourCells[m_Flock->m_gridRes*m_Flock->m_gridRes];


    for( int i = x - bucketRadius; i <= x + bucketRadius; ++i ){
        for( int j = y - bucketRadius; j <= y + bucketRadius; ++j ){
            if(i>=0 && j>=0 && i<=res-1 && j<= res-1)
            {

                //printf("bucket radius : %d cell: %d \n",bucketRadius, cell);
                //if((j*m_Flock->m_gridRes + i) != cell  )
                //{
                    neighbourCells[count] = (j*res) + i;

                    //std::cout<<neighbourCells[count]<<" neighbour cells \n";

                    count ++;

                //}
            }

        }
    }



}

__global__ void emptyCellCheck(unsigned int *neighbourCells,
                               unsigned int *cellOcc,
                              const unsigned int res)
{
    uint idx = blockIdx.x * blockDim.x + threadIdx.x;

    // Remove empty cells
    if(idx < res*res)
    {
//        printf("%d index \n", idx);

        if(neighbourCells[idx]<res*res)
        {

            //printf("%d neighbour cell \n", neighbourCells[idx]);
            // if cell empty
            if(cellOcc[neighbourCells[idx]] == 0)
            {
                //add points to neighbourhood

                //printf("deleting cell %d \n", neighbourCells[idx]);
                neighbourCells[idx] = NULL_CELL;

            }
        }
    }

}


// find cells surrounding current particles cells
__global__ void nearestNeighbourPnts(unsigned int *neighbourhood,
                                     unsigned int *neighbourCells,
                                     unsigned int *hash,
                                     const unsigned int N,
                                     const unsigned int res
                                )
{

    // Compute the index of this thread: i.e. the point we are testing
    uint idx = blockIdx.x * blockDim.x + threadIdx.x;
    uint idy = blockIdx.y * blockDim.y + threadIdx.y;

    //printf("%d x, %d y \n", idx, idy);

    if(idx<res*res)
    {
        // only check occupied cells
        if(neighbourCells[idx] < res*res)
        {
            if(idy < N)
            {
                if(neighbourCells[idx] == hash[idy] )
                {


                      neighbourhood[idy]=idy;

                }
            }


        }
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



#endif // NEARESTNEIGHBOUR_H
