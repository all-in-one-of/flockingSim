#include "flock_gpu.cuh"

//#include "prey_gpu.cuh"
#include <cuda.h>
#include <curand.h>

#include "flock_kernals.cuh"

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

    m_dCohesionX.resize(m_numBoids);
    m_dCohesionZ.resize(m_numBoids);

    m_dSeperationX.resize(m_numBoids);
    m_dSeperationZ.resize(m_numBoids);

    m_dAlignmentX.resize(m_numBoids);
    m_dAlignmentZ.resize(m_numBoids);



    // fill vector with random values for pos
    thrust::device_vector <float> tmp_PosPnts(NUM_BOIDS*4);
    float * tmp_PosPnts_ptr = thrust::raw_pointer_cast(&tmp_PosPnts[0]);
    randFloats(tmp_PosPnts_ptr, NUM_BOIDS*4);


    // give random start positions
    m_dBoidsPosX.assign(tmp_PosPnts.begin(), tmp_PosPnts.begin() + NUM_BOIDS);
    m_dBoidsPosZ.assign(tmp_PosPnts.begin() + NUM_BOIDS, tmp_PosPnts.begin() + 2*NUM_BOIDS);

    // give random start vel
    m_dBoidsVelX.assign(tmp_PosPnts.begin() + 2*NUM_BOIDS, tmp_PosPnts.begin() + 3*NUM_BOIDS);
    m_dBoidsVelZ.assign(tmp_PosPnts.begin() + 3*NUM_BOIDS, tmp_PosPnts.begin() + 4*NUM_BOIDS);



    // create pointers pointing to the device vectors
    m_dBoidsPosX_ptr= thrust::raw_pointer_cast(&m_dBoidsPosX[0]);
    m_dBoidsPosZ_ptr= thrust::raw_pointer_cast(&m_dBoidsPosZ[0]);

    m_dBoidsVelX_ptr= thrust::raw_pointer_cast(&m_dBoidsVelX[0]);
    m_dBoidsVelZ_ptr= thrust::raw_pointer_cast(&m_dBoidsVelZ[0]);

    m_dCohesionX_ptr= thrust::raw_pointer_cast(&m_dCohesionX[0]);
    m_dCohesionZ_ptr= thrust::raw_pointer_cast(&m_dCohesionZ[0]);

    m_dSeperationX_ptr= thrust::raw_pointer_cast(&m_dSeperationX[0]);
    m_dSeperationZ_ptr= thrust::raw_pointer_cast(&m_dSeperationZ[0]);

    m_dAlignmentX_ptr= thrust::raw_pointer_cast(&m_dAlignmentX[0]);
    m_dAlignmentZ_ptr= thrust::raw_pointer_cast(&m_dAlignmentZ[0]);



}

Flock_GPU::~Flock_GPU()
{

}

/// @brief a method to update each of the Boids contained in the system
void Flock_GPU::update()
{




        unsigned int nThreads = 1024;
        unsigned int nBlocks = m_numBoids/ nThreads + 1;

        //thrust::device_vector<unsigned int> d_numNeighbourBoids(GRID_RESOLUTION*GRID_RESOLUTION, NULL_CELL);


        //unsigned int * d_numNeighbourBoids_ptr = thrust::raw_pointer_cast(&d_numNeighbourBoids[0]);

        // for nearest neighbour
        unsigned int blockN = NUM_BOIDS/32 + 1;
        dim3 block2(32, 32); // block of (X,Y) threads
        dim3 grid2(blockN, 1); // grid blockN * blockN blocks

        // reset vectors
        thrust::fill(m_dCohesionX.begin(), m_dCohesionX.begin() + m_numBoids, 0);
        thrust::fill(m_dCohesionZ.begin(), m_dCohesionZ.begin() + m_numBoids, 0);

        thrust::fill(m_dSeperationX.begin(), m_dSeperationX.begin() + m_numBoids, 0);
        thrust::fill(m_dSeperationZ.begin(), m_dSeperationZ.begin() + m_numBoids, 0);

        thrust::fill(m_dAlignmentX.begin(), m_dAlignmentX.begin() + m_numBoids, 0);
        thrust::fill(m_dAlignmentZ.begin(), m_dAlignmentZ.begin() + m_numBoids, 0);



        flock_kernal<<<grid2,block2>>>(m_dCohesionX_ptr, m_dCohesionZ_ptr, m_dSeperationX_ptr, m_dSeperationZ_ptr, m_dAlignmentX_ptr, m_dAlignmentZ_ptr, m_dBoidsPosX_ptr, m_dBoidsPosZ_ptr, m_dBoidsVelX_ptr, m_dBoidsVelZ_ptr);

        cudaThreadSynchronize();

        limitVel_kernal<<<nBlocks,nThreads>>>(0.02, m_dBoidsVelX_ptr, m_dBoidsVelZ_ptr);

        cudaThreadSynchronize();

        avoidBoundaries_kernal<<<nBlocks,1024>>>(m_dBoidsPosX_ptr, m_dBoidsPosZ_ptr, m_dBoidsVelX_ptr, m_dBoidsVelZ_ptr);


        cudaThreadSynchronize();


        updatePos_kernal<<<nBlocks,1024>>>(m_dBoidsPosX_ptr, m_dBoidsPosZ_ptr, m_dBoidsVelX_ptr, m_dBoidsVelZ_ptr);

        cudaThreadSynchronize();





}


void Flock_GPU::dumpGeo(uint _frameNumber)
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
    for(unsigned int i=0; i<m_numBoids; ++i)
    {


        ss<<m_dBoidsPosX[i]<<" "<<0<<" "<<m_dBoidsPosZ[i] << " 1 ";

        ss<<"("<<std::abs(1)<<" "<<std::abs(1)<<" "<<std::abs(1)<<")\n";
    }

    // now write out the index values
    ss<<"PrimitiveAttrib\n";
    ss<<"generator 1 index 1 location1\n";
    ss<<"dopobject 1 index 1 /obj/AutoDopNetwork:1\n";
    ss<<"Part "<<m_numBoids<<" ";
    for(size_t i=0; i<m_numBoids; ++i)
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
