#include "flock_gpu.cuh"
//#include "boidFactory.h"

//#include "prey_gpu.cuh"
#include <cuda.h>
#include <curand.h>

#include "nearestneighbour_gpu.cuh"

#include<sys/time.h>
#include <sstream>
#include <iostream>
#include <fstream>

#include "random.cuh"

/// Used to define a point not in the neighbourhood
#define NULL_PNT UINT_MAX

#define CURAND_CALL(x) do { if((x)!=CURAND_STATUS_SUCCESS) { \
    printf("Error at %s:%d\n",__FILE__,__LINE__);\
    return EXIT_FAILURE;}} while(0)

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

    // cant set size when constructing as member variable so resize here instead
    m_dBoidsPosX.resize(m_numBoids);
    m_dBoidsPosZ.resize(m_numBoids);

    m_dBoidsVelX.resize(m_numBoids);
    m_dBoidsVelZ.resize(m_numBoids);

    m_dHash.resize(m_numBoids);
    m_dCellOcc.resize(m_gridRes * m_gridRes);

    m_dneighbourPnts.resize(m_numBoids, NULL_PNT);


    // fill vector with random values for pos
    thrust::device_vector <float> tmp_PosPnts(NUM_POINTS*2);
    float * tmp_PosPnts_ptr = thrust::raw_pointer_cast(&tmp_PosPnts[0]);
    randFloats(tmp_PosPnts_ptr, NUM_POINTS*2);

    // fill vector with random values for vel
    thrust::device_vector <float> tmp_VelPnts(NUM_POINTS*2);
    float * tmp_VelPnts_ptr = thrust::raw_pointer_cast(&tmp_VelPnts[0]);
    randFloats(tmp_VelPnts_ptr, NUM_POINTS*2);



    // give random start positions
    m_dBoidsPosX.assign(tmp_PosPnts.begin(), tmp_PosPnts.begin() + NUM_POINTS);
    m_dBoidsPosZ.assign(tmp_PosPnts.begin() + NUM_POINTS, tmp_PosPnts.begin() + 2*NUM_POINTS);

    // give random start vel
    m_dBoidsVelX.assign(tmp_VelPnts.begin(), tmp_VelPnts.begin() + NUM_POINTS);
    m_dBoidsVelZ.assign(tmp_VelPnts.begin() + NUM_POINTS, tmp_VelPnts.begin() + 2*NUM_POINTS);






    // create pointers pointing to the device vectors
    m_dBoidsPosX_ptr= thrust::raw_pointer_cast(&m_dBoidsPosX[0]);
    m_dBoidsPosZ_ptr= thrust::raw_pointer_cast(&m_dBoidsPosZ[0]);

    m_dBoidsVelX_ptr= thrust::raw_pointer_cast(&m_dBoidsVelX[0]);
    m_dBoidsVelZ_ptr= thrust::raw_pointer_cast(&m_dBoidsVelZ[0]);


    m_dHash_ptr= thrust::raw_pointer_cast(&m_dHash[0]);
    m_dCellOcc_ptr= thrust::raw_pointer_cast(&m_dCellOcc[0]);

    m_dneighbourPnts_ptr = thrust::raw_pointer_cast(&m_dneighbourPnts[0]);



    // give random start positions
//    thrust::fill(m_dBoidsPosX.begin(), m_dBoidsPosX.begin() + m_numBoids, ((float(rand())/RAND_MAX)-0.5)*4);

//    thrust::fill(m_dBoidsPosZ.begin(), m_dBoidsPosZ.begin() + m_numBoids, ((float(rand())/RAND_MAX)-0.5)*4);

//    // give random initial velocities
//    thrust::fill(m_dBoidsPosX.begin(), m_dBoidsPosX.begin() + m_numBoids, ((float(rand())/RAND_MAX)-0.5)*4);

//    thrust::fill(m_dBoidsPosZ.begin(), m_dBoidsPosZ.begin() + m_numBoids, ((float(rand())/RAND_MAX)-0.5)*4);




//    randFloats(m_dBoidsPosX_ptr, NUM_POINTS);
//    randFloats(m_dBoidsPosZ_ptr, NUM_POINTS);

    //randFloats(m_dBoidsVelX_ptr, NUM_POINTS);
    //randFloats(m_dBoidsVelZ_ptr, NUM_POINTS);




    //m_Boids.resize(m_numBoids);

    //BoidFactory *b = new BoidFactory;

    //unsigned int nThreads = 1024;
    //unsigned int nBlocks = NUM_POINTS / nThreads + 1;


    for (int i=0; i< _numBoids; ++i)
    {

        m_Boids.push_back(Prey_GPU(this,i));



        //setPositions<<<nBlocks, nThreads>>>(m_dPos_ptr, m_Boids[i].getPos().x, i);


        //Prey_GPU * prey = new Prey_GPU(this,i);





//         m_dBoidsPosX[i]=m_Boids[i].getPos()[0];
//         m_dBoidsPosZ[i]=m_Boids[i].getPos()[2];

         // make positions between 0-1 rather then -2 to 2
         //m_dBoidsPosX[i] = (1.0f/4.0f)*(m_Boids[i].getPos()[0] + 2);
         //m_dBoidsPosZ[i] = (1.0f/4.0f)*(m_Boids[i].getPos()[2] + 2);

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


    //if(m_frame_count < 300)
    //{
        //std::cout<<"dump frame \n";
        dumpGeo(m_frame_count,getBoidsVector());
        m_frame_count ++;
    //}

    //hash();
    //cellOcc();

    //findNeighbours(0.24,0);


    for(int i=0; i<m_numBoids; ++i)
    {

        m_Boids[i].update();

        // make positions between 0-1 rather then -2 to 2
        //m_dBoidsPosX[i] = (1.0f/4.0f)*(m_Boids[i].getPos()[0] + 2);
        //m_dBoidsPosZ[i] = (1.0f/4.0f)*(m_Boids[i].getPos()[2] + 2);

    }


}


void Flock_GPU::findNeighbours(float _neighbourhoodDist, int _boidID)
{
    // fill vectors with boid positions
    // make positions between 0-1 rather then -2 to 2
   // m_dBoidsPosX[i] = (1.0f/4.0f)*(m_Boids[i].getPos()[0] + 2);
    //m_dBoidsPosZ[i] = (1.0f/4.0f)*(m_Boids[i].getPos()[2] + 2);

    thrust::fill(m_dBoidsPosX.begin(), m_dBoidsPosX.begin() + m_numBoids, float(rand())/RAND_MAX);

    thrust::fill(m_dBoidsPosZ.begin(), m_dBoidsPosZ.begin() + m_numBoids, float(rand())/RAND_MAX);

    // divide by grid resolution as grid 0-1 and boids plane -2 - 2
    _neighbourhoodDist /= (2 * m_gridRes);


    thrust::fill(m_dneighbourPnts.begin(), m_dneighbourPnts.begin() + m_numBoids, NULL_PNT);
    // reset vector values to null_pnt
    //m_dneighbourPnts.clear();
    //m_dneighbourPnts.resize(m_numBoids,NULL_PNT);






        //Flock_GPU *flock = new Flock_GPU(20);

        //Prey_GPU *prey = new Prey_GPU(flock,1);



        // First thing is we'll generate a big old vector of random numbers for the purposes of
        // fleshing out our point data. This is much faster to do in one step than 3 seperate
        // steps.
//        thrust::device_vector<float> d_Rand(NUM_POINTS*3);


//        float * d_Rand_ptr = thrust::raw_pointer_cast(&d_Rand[0]);


//        randFloats(d_Rand_ptr, NUM_POINTS*3);



        // We'll store the components of the 3d vectors in separate arrays.
        // This 'structure of arrays' (SoA) approach is usually more efficient than the
        // 'array of structures' (AoS) approach.  The primary reason is that structures,
        // like Float3, don't always obey the memory coalescing rules, so they are not
        // efficiently transferred to and from memory.  Another reason to prefer SoA to
        // AoS is that we don't aways want to process all members of the structure.  For
        // example, if we only need to look at first element of the structure then it
        // is wasteful to load the entire structure from memory.  With the SoA approach,
        // we can chose which elements of the structure we wish to read.
        //thrust::device_vector<float> d_Px(d_Rand.begin(), d_Rand.begin()+NUM_POINTS);
        //thrust::device_vector<float> d_Py(d_Rand.begin()+NUM_POINTS, d_Rand.begin()+2*NUM_POINTS);
    //    thrust::device_vector<float> d_Pz(d_Rand.begin()+2*NUM_POINTS, d_Rand.end());

        // This vector will hold the grid cell occupancy (set to zero)
        //thrust::device_vector<unsigned int> d_cellOcc(GRID_RESOLUTION*GRID_RESOLUTION, 0);

        // This vector will hold our hash values, one for each point
        //thrust::device_vector<unsigned int> d_hash(NUM_POINTS);
        //thrust::copy(d_hash.begin(), d_hash.end(), std::ostream_iterator<unsigned int>(std::cout, " "));

        // Vector storing neighbour pnts idx
        //thrust::device_vector<unsigned int> d_neighbours(NUM_POINTS,NULL_PNT);

        // Holds neighbour cells
        thrust::device_vector<unsigned int> d_neighbourCells(GRID_RESOLUTION*GRID_RESOLUTION, NULL_CELL);

        // Typecast some raw pointers to the data so we can access them with CUDA functions
        //unsigned int * d_hash_ptr = thrust::raw_pointer_cast(&d_hash[0]);
        //unsigned int * d_cellOcc_ptr = thrust::raw_pointer_cast(&d_cellOcc[0]);
        //unsigned int * d_neighbours_ptr = thrust::raw_pointer_cast(&d_neighbours[0]);
        unsigned int * d_neighbourCells_ptr = thrust::raw_pointer_cast(&d_neighbourCells[0]);
        //float * d_Px_ptr = thrust::raw_pointer_cast(&d_Px[0]);
        //float * d_Py_ptr = thrust::raw_pointer_cast(&d_Py[0]);
    //   float * d_Pz_ptr = thrust::raw_pointer_cast(&d_Pz[0]);

        // The number of threads per blockshould normally be determined from your hardware, but 1024
        // is pretty standard. Remember that each block will be assigned to a single SM, with it's
        // own local memory.
        unsigned int nThreads = 1024;
        unsigned int nBlocks = NUM_POINTS / nThreads + 1;


        //dim3 threadsPerBlock(8, 8);
        //dim3 numBlocks(GRID_RESOLUTION/threadsPerBlock.x, GRID_RESOLUTION/threadsPerBlock.y);

         //int blockDim = 1024 / GRID_RESOLUTION + 1; // 9 threads per block
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

//        // The special CUDA syntax below executes our parallel function with the specified parameters
//        // using the number of blocks and threads provided.
//        pointHash<<<nBlocks, nThreads>>>(m_dHash_ptr, m_dBoidsPosX_ptr, m_dBoidsPosZ_ptr,
//                                         NUM_POINTS,
//                                         GRID_RESOLUTION);

//        // Make sure all threads have wrapped up before completing the timings
//        cudaThreadSynchronize();

//        // Now we can sort our points to ensure that points in the same grid cells occupy contiguous memory
//        thrust::sort_by_key(m_dHash.begin(), m_dHash.end(),
//                            thrust::make_zip_iterator(
//                                thrust::make_tuple( m_dBoidsPosX.begin(), m_dBoidsPosZ.begin())));

//        // Make sure all threads have wrapped up before completing the timings
//        cudaThreadSynchronize();

//        // Now we can count the number of points in each grid cell
//        countCellOccupancy<<<nBlocks, nThreads>>>(m_dCellOcc_ptr, m_dHash_ptr, m_dCellOcc.size(), m_dHash.size());

//        // Make sure all threads have wrapped up before completing the timings
//        cudaThreadSynchronize();

        neighbourhoodCells<<<grid, block>>>(d_neighbourCells_ptr,_neighbourhoodDist,GRID_RESOLUTION, m_dHash[_boidID]);

        cudaThreadSynchronize();

        // sort into order
        thrust::sort(d_neighbourCells.begin(), d_neighbourCells.end());

        //emptyCellCheck<<<nBlocks,nThreads>>>(d_neighbourCells_ptr, d_cellOcc_ptr,GRID_RESOLUTION);

        //cudaThreadSynchronize();

        // sort into order again
        //thrust::sort(d_neighbourCells.begin(), d_neighbourCells.end());

        // Finds pnt index in neighbourhood cells
        nearestNeighbourPnts<<<grid2, block2>>>(m_dneighbourPnts_ptr, d_neighbourCells_ptr, m_dHash_ptr,NUM_POINTS,GRID_RESOLUTION);

        cudaThreadSynchronize();

        // sort into order finally (PROBABLY UNNECCESERY!)
        thrust::sort(m_dneighbourPnts.begin(), m_dneighbourPnts.end());




        gettimeofday(&tim, NULL);
        t2=tim.tv_sec+(tim.tv_usec/1000000.0);
        //std::cout << "Grid sorted "<<NUM_POINTS<<" points into grid of "<<GRID_RESOLUTION*GRID_RESOLUTION*GRID_RESOLUTION<<" cells in " << t2-t1 << "s\n";

        // Only dump the debugging information if we have a manageable number of points.
//        if (NUM_POINTS <= 100) {

//            std::cout << "Boid: "<<_boidID<<"\n";
//            std::cout << "Boid Cell: "<<m_dHash[_boidID]<<"\n";
//            thrust::copy(m_dBoidsPosX.begin(), m_dBoidsPosX.end(), std::ostream_iterator<float>(std::cout, " "));
//            std::cout << "\n\n";
//            thrust::copy(m_dBoidsPosZ.begin(), m_dBoidsPosZ.end(), std::ostream_iterator<float>(std::cout, " "));
//            std::cout << "\n\n";
//            thrust::copy(m_dHash.begin(), m_dHash.end(), std::ostream_iterator<unsigned int>(std::cout, " "));
//            std::cout << "\n\n";
//            thrust::copy(m_dCellOcc.begin(), m_dCellOcc.end(), std::ostream_iterator<unsigned int>(std::cout, " "));
//            std::cout << "\n\n";
//            thrust::copy(d_neighbourCells.begin(), d_neighbourCells.end(), std::ostream_iterator<unsigned int>(std::cout, " "));
//            std::cout << "\n\n";
//            thrust::copy(m_dneighbourPnts.begin(), m_dneighbourPnts.end(), std::ostream_iterator<unsigned int>(std::cout, " "));
//            std::cout << "\n\n";

//        }
        //return 0;



}

/// @brief a method to draw all the Boids contained in the system
void Flock_GPU::draw()
{



}

void Flock_GPU::hash()
{

    unsigned int nThreads = 1024;
    unsigned int nBlocks = NUM_POINTS / nThreads + 1;

    // The special CUDA syntax below executes our parallel function with the specified parameters
    // using the number of blocks and threads provided.
    pointHash<<<nBlocks, nThreads>>>(m_dHash_ptr, m_dBoidsPosX_ptr, m_dBoidsPosZ_ptr,
                                     NUM_POINTS,
                                     GRID_RESOLUTION);

    // Make sure all threads have wrapped up before completing the timings
    cudaThreadSynchronize();

    // Now we can sort our points to ensure that points in the same grid cells occupy contiguous memory
    thrust::sort_by_key(m_dHash.begin(), m_dHash.end(),
                        thrust::make_zip_iterator(
                            thrust::make_tuple( m_dBoidsPosX.begin(), m_dBoidsPosZ.begin())));

    // Make sure all threads have wrapped up before completing the timings
    cudaThreadSynchronize();

}


void Flock_GPU::dumpGeo(uint _frameNumber, std::vector<Prey_GPU> _boids)
{
    char fname[150];

    std::sprintf(fname,"geo/flock_gpu.%03d.geo",++_frameNumber);
    // we will use a stringstream as it may be more efficient
    std::stringstream ss;
    std::ofstream file;
    file.open(fname);
    if (!file.is_open())
    {
        std::cerr << "failed to Open file "<<fname<<'\n';
        exit(EXIT_FAILURE);
    }
    // write header see here http://www.sidefx.com/docs/houdini15.0/io/formats/geo
    ss << "PGEOMETRY V5\n";
    ss << "NPoints " << getNoBoids() << " NPrims 1\n";
    ss << "NPointGroups 0 NPrimGroups 1\n";
    // this is hard coded but could be flexible we have 1 attrib which is Colour
    ss << "NPointAttrib 1  NVertexAttrib 0 NPrimAttrib 2 NAttrib 0\n";
    // now write out our point attrib this case Cd for diffuse colour
    ss <<"PointAttrib \n";
    // default the colour to white
    ss <<"Cd 3 float 1 1 1\n";
    // now we write out the particle data in the format
    // x y z 1 (attrib so in this case colour)
    for(unsigned int i=0; i<_boids.size(); ++i)
    {


        ss<<m_dBoidsPosX[i]<<" "<<0<<" "<<m_dBoidsPosZ[i] << " 1 ";
        //ss<<"("<<_boids[i].cellCol.x<<" "<<_boids[i].cellCol.y<<" "<< _boids[i].cellCol.z<<")\n";
        ss<<"("<<std::abs(1)<<" "<<std::abs(1)<<" "<<std::abs(1)<<")\n";
    }

    // now write out the index values
    ss<<"PrimitiveAttrib\n";
    ss<<"generator 1 index 1 location1\n";
    ss<<"dopobject 1 index 1 /obj/AutoDopNetwork:1\n";
    ss<<"Part "<<_boids.size()<<" ";
    for(size_t i=0; i<_boids.size(); ++i)
    {
        ss<<i<<" ";
    }
    ss<<" [0	0]\n";
    ss<<"box_object1 unordered\n";
    ss<<"1 1\n";
    ss<<"beginExtra\n";
    ss<<"endExtra\n";
    // dump string stream to disk;
    file<<ss.rdbuf();
    file.close();



}

void Flock_GPU::cellOcc()
{

    thrust::fill(m_dCellOcc.begin(), m_dCellOcc.begin() + (m_gridRes*m_gridRes), 0);

    // reset vector values to 0
    //m_dCellOcc.clear();
    //m_dCellOcc.resize(m_gridRes*m_gridRes,0);
    unsigned int nThreads = 1024;
    unsigned int nBlocks = NUM_POINTS / nThreads + 1;

    // Now we can count the number of points in each grid cell
    countCellOccupancy<<<nBlocks, nThreads>>>(m_dCellOcc_ptr, m_dHash_ptr, m_dCellOcc.size(), m_dHash.size());

    // Make sure all threads have wrapped up before completing the timings
    cudaThreadSynchronize();

}

/**
 * Fill an array with random floats using the CURAND function.
 * \param devData The chunk of memory you want to fill with floats within the range (0,1]
 * \param n The size of the chunk of data
 */
int Flock_GPU::randFloats(float *&devData, const size_t n) {

    // The generator, used for random numbers
    curandGenerator_t gen;

    // Create pseudo-random number generator
    CURAND_CALL(curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT));

    // Set seed to be the current time (note that calls close together will have same seed!)
    CURAND_CALL(curandSetPseudoRandomGeneratorSeed(gen, time(NULL)));

    // Generate n floats on device
    CURAND_CALL(curandGenerateUniform(gen, devData, n));

    // Cleanup
    CURAND_CALL(curandDestroyGenerator(gen));
    return EXIT_SUCCESS;
}
