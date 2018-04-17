#include "flock_gpu.cuh"
//#include "boidFactory.h"

#include "prey_gpu.cuh"


#include "nearestneighbour_gpu.cuh"

#include<sys/time.h>

#include "random.cuh"

//__global__ void setPositions(float * posVec, float _currentPos, int _id)
//{
//    posVec[_id] = _currentPos;

//}

/// @brief ctor
/// @param _pos the position of the Flock
/// @param _numBoids the number of Boids to create
Flock_GPU::Flock_GPU(int _numBoids )
{


    m_numBoids=_numBoids;

    //thrust::device_vector<float> d_Pos(m_numBoids*3);

    m_dPosX.resize(m_numBoids);
    m_dPosY.resize(m_numBoids);


    m_dPosX_ptr= thrust::raw_pointer_cast(&m_dPosX[0]);
    m_dPosY_ptr= thrust::raw_pointer_cast(&m_dPosY[0]);








    //m_Boids.resize(m_numBoids);

    //BoidFactory *b = new BoidFactory;

    unsigned int nThreads = 1024;
    unsigned int nBlocks = NUM_POINTS / nThreads + 1;


    for (int i=0; i< _numBoids; ++i)
    {

        m_Boids.push_back(Prey_GPU(this,i));

        //setPositions<<<nBlocks, nThreads>>>(m_dPos_ptr, m_Boids[i].getPos().x, i);


        //Prey_GPU * prey = new Prey_GPU(this,i);





         m_dPosX[i]=m_Boids[i].getPos().x;
         m_dPosY[i]=m_Boids[i].getPos().z;

         // make positions between 0-1 rather then -2 to 2
         m_dPosX[i] = (1.0f/4.0f)*(m_dPosX[i] + 2);
         m_dPosY[i] = (1.0f/4.0f)*(m_dPosY[i] + 2);

//        std::cout<<m_dPos[(3*i)]<<" "<<m_dPos[(3*i)+1]<<" "<<m_dPos[(3*i)+2]<<" \n";

    }

    //m_dneighbourPnts_ptr =  thrust::raw_pointer_cast(&m_dneighbourPnts[0]);

    ///Predator(this,_numBoids+1);

    //delete b;

}

Flock_GPU::~Flock_GPU()
{

}

void Flock_GPU::createBoidsMesh()
{
//    for(int i = 0; i< m_numBoids; i++)
//    {

//    }
}

/// @brief a method to update each of the Boids contained in the system
void Flock_GPU::update()
{



    for(int i=0; i<m_numBoids; ++i)
    {






        hash();
        cellOcc();



        m_Boids[i].update();


    }


}


unsigned int * Flock_GPU::findNeighbours(float _neighbourhoodDist, int _boidID)
{




        //Flock_GPU *flock = new Flock_GPU(20);

        //Prey_GPU *prey = new Prey_GPU(flock,1);



        // First thing is we'll generate a big old vector of random numbers for the purposes of
        // fleshing out our point data. This is much faster to do in one step than 3 seperate
        // steps.
        thrust::device_vector<float> d_Rand(NUM_POINTS*3);


        float * d_Rand_ptr = thrust::raw_pointer_cast(&d_Rand[0]);


        //randFloats(d_Rand_ptr, NUM_POINTS*3);



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
        thrust::device_vector<unsigned int> d_neighbours(NUM_POINTS,NULL_PNT);

        // Holds neighbour cells
        thrust::device_vector<unsigned int> d_neighbourCells(GRID_RESOLUTION*GRID_RESOLUTION, NULL_CELL);

        // Typecast some raw pointers to the data so we can access them with CUDA functions
        unsigned int * d_hash_ptr = thrust::raw_pointer_cast(&d_hash[0]);
        unsigned int * d_cellOcc_ptr = thrust::raw_pointer_cast(&d_cellOcc[0]);
        unsigned int * d_neighbours_ptr = thrust::raw_pointer_cast(&d_neighbours[0]);
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
        pointHash<<<nBlocks, nThreads>>>(d_hash_ptr, m_dPosX_ptr, m_dPosY_ptr,
                                         NUM_POINTS,
                                         GRID_RESOLUTION);

        // Make sure all threads have wrapped up before completing the timings
        cudaThreadSynchronize();

        // Now we can sort our points to ensure that points in the same grid cells occupy contiguous memory
        thrust::sort_by_key(d_hash.begin(), d_hash.end(),
                            thrust::make_zip_iterator(
                                thrust::make_tuple( m_dPosX.begin(), m_dPosX.begin())));

        // Make sure all threads have wrapped up before completing the timings
        cudaThreadSynchronize();

        // Now we can count the number of points in each grid cell
        countCellOccupancy<<<nBlocks, nThreads>>>(d_cellOcc_ptr, d_hash_ptr, d_cellOcc.size(), d_hash.size());

        // Make sure all threads have wrapped up before completing the timings
        cudaThreadSynchronize();

        neighbourhoodCells<<<grid, block>>>(d_neighbourCells_ptr,_neighbourhoodDist,GRID_RESOLUTION, 0);

        cudaThreadSynchronize();

        // sort into order
        thrust::sort(d_neighbourCells.begin(), d_neighbourCells.end());

        //emptyCellCheck<<<nBlocks,nThreads>>>(d_neighbourCells_ptr, d_cellOcc_ptr,GRID_RESOLUTION);

        //cudaThreadSynchronize();

        // sort into order again
        //thrust::sort(d_neighbourCells.begin(), d_neighbourCells.end());

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

            thrust::copy(m_dPosX.begin(), m_dPosX.end(), std::ostream_iterator<float>(std::cout, " "));
            std::cout << "\n\n";
            thrust::copy(m_dPosY.begin(), m_dPosY.end(), std::ostream_iterator<float>(std::cout, " "));
            std::cout << "\n\n";
            thrust::copy(d_neighbourCells.begin(), d_neighbourCells.end(), std::ostream_iterator<unsigned int>(std::cout, " "));
            std::cout << "\n\n";
            thrust::copy(d_neighbours.begin(), d_neighbours.end(), std::ostream_iterator<unsigned int>(std::cout, " "));
            std::cout << "\n\n";
            thrust::copy(d_hash.begin(), d_hash.end(), std::ostream_iterator<unsigned int>(std::cout, " "));
            std::cout << "\n\n";
            thrust::copy(d_cellOcc.begin(), d_cellOcc.end(), std::ostream_iterator<unsigned int>(std::cout, " "));
        }
        //return 0;



}

/// @brief a method to draw all the Boids contained in the system
void Flock_GPU::draw()
{
    for(int i=0; i<m_numBoids; ++i)
    {
        m_Boids[i].draw();
    }


}

void Flock_GPU::hash()
{
    // reset and recaulculate each turn
    m_hashVec.clear();

    int res = m_gridRes;
    for(int i = 0; i< m_numBoids; i++)
    {
        glm::vec3 pos = m_Boids[i].getPos();

        //std::cout<<pos.m_x<<" "<<pos.m_z<<" original \n";

        // make position between 0-1 rather then -3 - 3
        float posx = pos[0];
        float posz = pos[2];
        posx = (1.0f/8.0f)*(posx + 4);
        posz = (1.0f/8.0f)*(posz + 4);

        //std::cout<<posx<<" "<<posz<<" altered \n";


        // 2d grid
        int gridPos[2];
        gridPos[0]=floor(posx * res);
        gridPos[1]=floor(posz * res);

        //std::cout<<gridPos[0]<<" "<<gridPos[1]<<" grid \n";



        // Test to see if all of the points are inside the grid
        bool isInside = true;
        unsigned int j;
        for (j=0; j<2; ++j)
            if ((gridPos[j] < 0) || (gridPos[j] >= res)) {
                isInside = false;
            }

        // Write out the hash value if the point is within range [0,1], else write NULL_HASH
        if (isInside) {
            m_hashVec.push_back( gridPos[0] + (res * gridPos[1]));
        } else {
            m_hashVec.push_back( NULL);
        }

        //std::cout<<m_hashVec[i]<<" hash \n";


    }

}

void Flock_GPU::cellOcc()
{
    //reset cellOcc array
    for(int i = 0; i<m_gridRes*m_gridRes; i++)
    {
        m_cellOcc[i] = 0;
    }

    for(int i = 0; i<m_numBoids; i++)
    {

        //std::cout<<m_hashVec[i]<<" hash \n";
        //std::cout<<m_cellOcc[m_hashVec[i]]<<" Cells \n";
        m_cellOcc[m_hashVec[i]] +=1;


        //std::cout<<m_cellOcc[m_hashVec[i]]<<" Cells now \n";



    }

//    for(int i = 0; i<m_gridRes*m_gridRes; i++)
//    {
//        //std::cout<<m_cellOcc[i]<<" Cells \n";


//    }

    //std::cout<<"done \n";

}
