
#include <cuda_runtime.h>
#include <cuda.h>
#include <curand.h>
#include <cuda_runtime_api.h>
#include <device_functions.h>


#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>

#include<sys/time.h>
#include <sstream>
#include <iostream>
#include <fstream>



#include <iostream>
#include <stdio.h>

// printf() is only supported
// for devices of compute capability 2.0 and higher
#if defined(__CUDA_ARCH__) && (__CUDA_ARCH__ < 200)
   #define printf(f, ...) ((void)(f, __VA_ARGS__),0)
#endif

#define CURAND_CALL(x) do { if((x)!=CURAND_STATUS_SUCCESS) { \
    printf("Error at %s:%d\n",__FILE__,__LINE__);\
    return EXIT_FAILURE;}} while(0)

/// The number of points to generate within 0,1
#define NUM_BOIDS 1000

#define NUM_FRAMES 150

__device__ float vectorMag_kernal(float  _vector1, float  _vector2, float  _vector3)
{
    float mag;

    mag = sqrtf((_vector1*_vector1) + (_vector2*_vector2) + (_vector3*_vector3));

    return mag;

}

__device__ void steerBoid_kernal(float * _targetX, float * _targetZ, float * _currentX, float * _currentZ, float * _sourceX, float *_sourceZ)
{
    uint idx = blockIdx.x * blockDim.x + threadIdx.x;

    float steeringForce = 0.02;


    float diffX[NUM_BOIDS];
    float diffZ[NUM_BOIDS];


   diffX[idx] = _targetX[idx] - _currentX[idx];
   diffZ[idx] = _targetZ[idx] - _currentZ[idx];



    _sourceX[idx] =( (diffX[idx]/vectorMag_kernal(diffX[idx], 0, diffZ[idx]))*steeringForce);
    _sourceZ[idx] =( (diffZ[idx]/vectorMag_kernal(diffX[idx], 0, diffZ[idx]))*steeringForce);

}

__global__ void avoidBoundaries_kernal(float * _posx, float * _posz, float * _velx, float * _velz, int _noBoids)
{


    uint idx = blockIdx.x * blockDim.x + threadIdx.x;

    float desiredVelX[NUM_BOIDS];
    float desiredVelZ[NUM_BOIDS];

    float * desiredVelX_ptr = &desiredVelX[0];
    float * desiredVelZ_ptr = &desiredVelZ[0];


    if(idx<_noBoids)
    {

        if(_posz[idx] >= 2 && _velz[idx] >0)
        {

            desiredVelX[idx] = _velx[idx];
            desiredVelZ[idx] = -_velz[idx];

            steerBoid_kernal(desiredVelX_ptr, desiredVelZ_ptr, _velx, _velz, _velx, _velz);


           // _velz[idx] = -_velz[idx];


        }
        else if(_posz[idx] <= -2 && _velz[idx] <0)
        {
            desiredVelX[idx] = _velx[idx];
            desiredVelZ[idx] = -_velz[idx];


            steerBoid_kernal(desiredVelX_ptr, desiredVelZ_ptr, _velx, _velz, _velx, _velz);

            //_velz[idx] = -_velz[idx];

        }
        else if(_posx[idx] >= 2 && _velx[idx] >0)
        {

            desiredVelX[idx] = -_velx[idx];
            desiredVelZ[idx] = _velz[idx];
            steerBoid_kernal(desiredVelX_ptr, desiredVelZ_ptr, _velx, _velz, _velx, _velz);

            //_velx[idx] = -_velx[idx];


        }
        else if(_posx[idx] <= -2 && _velx[idx] <0)
        {

            desiredVelX[idx] = -_velx[idx];
            desiredVelZ[idx] = _velz[idx];

            steerBoid_kernal(desiredVelX_ptr, desiredVelZ_ptr, _velx, _velz, _velx, _velz);
            //_velx[idx] = -_velx[idx];


        }
    }

}

__global__ void updatePos_kernal(float * _posx, float * _posz, float * _velx, float * _velz, int _noBoids)
{

    uint idx = blockIdx.x * blockDim.x + threadIdx.x;

    if(idx< _noBoids)
    {

    _posx[idx] +=  _velx[idx];
    _posz[idx] +=  _velz[idx];

    }



}






__device__ float distance_kernal(float  _posx, float  _posz, float  _otherPosx, float  _otherPosz)
{


    float distance = sqrtf(((_posx-_otherPosx)*(_posx-_otherPosx)) + ((_posz-_otherPosz)*(_posz-_otherPosz)));


    return distance;

}

__global__ void limitVel_kernal(float _limit, float * _posx, float * _posz, float * _velx, float * _velz, const int _noBoids)
{
    uint idx = blockIdx.x * blockDim.x + threadIdx.x;


    float mag[NUM_BOIDS];

    if(idx < _noBoids)
    {


        mag[idx] = sqrtf((_velx[idx]*_velx[idx]) + (_velz[idx]*_velz[idx]));




        if( mag[idx] > _limit)
        {


            _velx[idx] = (_velx[idx]/mag[idx])*_limit;
            _velz[idx] = (_velz[idx]/mag[idx])*_limit;




        }
    }


}



__device__ void alignment_kernal(float * _alignmentVectorX, float * _alignmentVectorZ, float * _posx, float * _posz, float * _velx, float * _velz, int _numBoids)
{

    int const noBoids = _numBoids;


     __shared__ unsigned int numberOfNeighbours[NUM_BOIDS];

     float tmpX[NUM_BOIDS];
     float tmpZ[NUM_BOIDS];

     float mag[NUM_BOIDS];

    // current boid whos neighbours were looking for
     uint idx = blockIdx.x * blockDim.x + threadIdx.x;
    // neighbours of current boid
    uint idy = blockIdx.y * blockDim.y + threadIdx.y;

    if(idx < noBoids && idy < noBoids)
    {

             // reset values
             numberOfNeighbours[idx] = 0;

             // wait for threads to sync
             __syncthreads();

                if(idx != idy)
                {

                    if(distance_kernal(_posx[idx], _posz[idx], _posx[idy], _posz[idy]) < 0.15)
                    {


                        atomicAdd(&(_alignmentVectorX[idx]), _velx[idy]);
                        atomicAdd(&(_alignmentVectorZ[idx]), _velz[idy]);


                        atomicAdd(&numberOfNeighbours[idx], 1);
                    }

                }

     }


        // wait for threads to sync
        __syncthreads();


        //limit to 1D
        if(idy == 0 && idx< noBoids)
        {

            //avoid dividing by zero
            if(numberOfNeighbours[idx] > 0)
            {


                //find average position
                _alignmentVectorX[idx] = _alignmentVectorX[idx]/numberOfNeighbours[idx];
                _alignmentVectorZ[idx] = _alignmentVectorZ[idx]/numberOfNeighbours[idx];


                // normalize
                mag[idx] = norm3d(_alignmentVectorX[idx], 0.0f, _alignmentVectorZ[idx]);

                if(mag[idx] > 0)
                {
                    _alignmentVectorX[idx] = (_alignmentVectorX[idx] / mag[idx]);
                    _alignmentVectorZ[idx] = (_alignmentVectorZ[idx] / mag[idx]);
                }


                //steer
                steerBoid_kernal(_alignmentVectorX, _alignmentVectorZ, _velx, _velz, _alignmentVectorX, _alignmentVectorZ);




            }

        }
}

__device__ void seperation_kernal(float * _seperationVectorX, float * _seperationVectorZ, float * _posx, float * _posz, float * _velx, float * _velz, int _numBoids)
{

    int const noBoids = _numBoids;

     __shared__ float _diffVectorX[NUM_BOIDS];
     __shared__ float _diffVectorZ[NUM_BOIDS];


     __shared__ unsigned int numberOfNeighbours[NUM_BOIDS];

     float tmpX[NUM_BOIDS];
     float tmpZ[NUM_BOIDS];

     float mag[NUM_BOIDS];


    // current boid whos neighbours were looking for
     uint idx = blockIdx.x * blockDim.x + threadIdx.x;
    // neighbours of current boid
    uint idy = blockIdx.y * blockDim.y + threadIdx.y;

    if(idx < noBoids && idy < noBoids)
    {


             // reset values
             numberOfNeighbours[idx] = 0;
             _diffVectorX[idx] = 0;
             _diffVectorZ[idx] = 0;

             // wait for threads to sync
             __syncthreads();


                if(idx != idy)
                {

                    if(distance_kernal(_posx[idx], _posz[idx], _posx[idy], _posz[idy]) < 0.1)
                    {

                        atomicAdd(&(_diffVectorX[idx]), (_posx[idy]-_posx[idx]));
                        atomicAdd(&(_diffVectorZ[idx]), (_posz[idy]-_posz[idx]));

                        // normalise (make atomic)
                        //_diffVectorX[idx] = _diffVectorX[idx] / norm3d(_diffVectorX[idx], 0.0f, _diffVectorZ[idx]);
                        //_diffVectorZ[idx] = _diffVectorZ[idx] / norm3d(_diffVectorX[idx], 0.0f, _diffVectorZ[idx]);

                        // add neighbours position to current boids part of the seperation vector
                        atomicAdd(&(_seperationVectorX[idx]), _diffVectorX[idx]);
                        atomicAdd(&(_seperationVectorZ[idx]), _diffVectorZ[idx]);




                        atomicAdd(&numberOfNeighbours[idx], 1);

                    }

                }

     }




        // wait for threads to sync
        __syncthreads();

        //limit to 1D
        if(idy == 0 && idx< noBoids)
        {

            //avoid dividing by zero
            if(numberOfNeighbours[idx] > 0)
            {



               // tmpX[idx] = _seperationVectorX[idx]/numberOfNeighbours[idx];
                //tmpZ[idx] = _seperationVectorZ[idx]/numberOfNeighbours[idx];


                //find average position
                _seperationVectorX[idx] = _seperationVectorX[idx]/numberOfNeighbours[idx];
                _seperationVectorZ[idx] = _seperationVectorZ[idx]/numberOfNeighbours[idx];



                _seperationVectorX[idx] = ( _seperationVectorX[idx] * -1);
                _seperationVectorZ[idx] = ( _seperationVectorZ[idx] * -1);


               mag[idx] = norm3d(_seperationVectorX[idx], 0.0f, _seperationVectorZ[idx]);

                if(mag[idx]>0)
                {
                    _seperationVectorX[idx] = (_seperationVectorX[idx] / mag[idx]);
                    _seperationVectorZ[idx] = (_seperationVectorZ[idx] / mag[idx]);
                }

                steerBoid_kernal(_seperationVectorX, _seperationVectorZ, _velx, _velz, _seperationVectorX, _seperationVectorZ);


            }



        }
}

__device__ void cohesion_kernal(float * _cohesionVectorX, float * _cohesionVectorZ, float * _posx, float * _posz, float * _velx, float * _velz, int _numBoids)
{

    int const noBoids = _numBoids;


     __shared__ unsigned int numberOfNeighbours[NUM_BOIDS];


     float mag[NUM_BOIDS];

    // current boid whos neighbours were looking for
     uint idx = blockIdx.x * blockDim.x + threadIdx.x;
    // neighbours of current boid
    uint idy = blockIdx.y * blockDim.y + threadIdx.y;

    if(idx < noBoids && idy < noBoids)
    {


             // reset values
             numberOfNeighbours[idx] = 0;
             _cohesionVectorX[idx] = 0;
             _cohesionVectorZ[idx] = 0;

             // wait for threads to sync
             __syncthreads();


                if(idx != idy)
                {

                    if(distance_kernal(_posx[idx], _posz[idx], _posx[idy], _posz[idy]) < 0.2)
                    {


                        // add neighbours position to current boids part of the cohesion vector
                        atomicAdd(&(_cohesionVectorX[idx]), _posx[idy]);
                        atomicAdd(&(_cohesionVectorZ[idx]), _posz[idy]);



                        atomicAdd(&numberOfNeighbours[idx], 1);


                    }

                }


     }




        // wait for threads to sync
        __syncthreads();


        //limit to 1D
        if(idy == 0 && idx< noBoids)
        {



            //avoid dividing by zero
            if(numberOfNeighbours[idx] > 0)
            {


                float tmpX = _cohesionVectorX[idx]/numberOfNeighbours[idx];
                float tmpZ = _cohesionVectorZ[idx]/numberOfNeighbours[idx];



                //find average position
                _cohesionVectorX[idx] = tmpX;
                _cohesionVectorZ[idx] = tmpZ;


                _cohesionVectorX[idx] = ( _cohesionVectorX[idx] - _posx[idx]);
                _cohesionVectorZ[idx] = ( _cohesionVectorZ[idx] - _posz[idx]);

                mag[idx] = norm3d(_cohesionVectorX[idx], 0.0f, _cohesionVectorZ[idx]);


                if(mag[idx] > 0)
                {
                    _cohesionVectorX[idx] = (_cohesionVectorX[idx] / mag[idx]);
                    _cohesionVectorZ[idx] = (_cohesionVectorZ[idx] / mag[idx]);

                }

                steerBoid_kernal(_cohesionVectorX, _cohesionVectorZ, _velx, _velz, _cohesionVectorX, _cohesionVectorZ);





            }

        }

}

__global__ void flock_kernal(float * _cohesionVectorX, float * _cohesionVectorZ,float * _seperationVectorX, float * _seperationVectorZ, float * _alignmentVectorX, float * _alignmentVectorZ, float * _posx, float * _posz, float * _velx, float * _velz, int _numBoids)
{
    uint idx = blockIdx.x * blockDim.x + threadIdx.x;
    uint idy = blockIdx.y * blockDim.y + threadIdx.y;


    float mag[NUM_BOIDS];



    if( idx <_numBoids)
    {

        // calculate cohesion
        cohesion_kernal(_cohesionVectorX, _cohesionVectorZ, _posx, _posz, _velx, _velz, _numBoids);

        seperation_kernal(_seperationVectorX, _seperationVectorZ, _posx, _posz, _velx, _velz, _numBoids);

        alignment_kernal(_alignmentVectorX, _alignmentVectorZ, _posx, _posz, _velx, _velz, _numBoids);


        // wait for threads to sync (dont add cohesion vector until calculated)
        __syncthreads();

        if(idy == 0)
        {

            _velx[idx]+=  _cohesionVectorX[idx] + _seperationVectorX[idx] +  _alignmentVectorX[idx];
            _velz[idx]+=  _cohesionVectorZ[idx] + _seperationVectorZ[idx] +  _alignmentVectorZ[idx];

        }
    }



}

void dumpGeo(uint _frameNumber, thrust::device_vector <float> _posX, thrust::device_vector <float> _posZ)
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
    ss << "NPoints " << NUM_BOIDS << " NPrims 1\n";
    ss << "NPointGroups 0 NPrimGroups 1\n";
    // this is hard coded but could be flexible we have 1 attrib which is Colour
    ss << "NPointAttrib 1  NVertexAttrib 0 NPrimAttrib 2 NAttrib 0\n";
    // now write out our point attrib this case Cd for diffuse colour
    ss <<"PointAttrib \n";
    // default the colour to white
    ss <<"Cd 3 float 1 1 1\n";
    // now we write out the particle data in the format
    // x y z 1 (attrib so in this case colour)
    for(unsigned int i=0; i<NUM_BOIDS; ++i)
    {


        ss<<_posX[i]<<" "<<0<<" "<<_posZ[i] << " 1 ";

        ss<<"("<<std::abs(1)<<" "<<std::abs(1)<<" "<<std::abs(1)<<")\n";
    }

    // now write out the index values
    ss<<"PrimitiveAttrib\n";
    ss<<"generator 1 index 1 location1\n";
    ss<<"dopobject 1 index 1 /obj/AutoDopNetwork:1\n";
    ss<<"Part "<<NUM_BOIDS<<" ";
    for(size_t i=0; i<NUM_BOIDS; ++i)
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
 * \author Richard Southern
 */
int randFloats(float *&devData, const size_t n) {

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


int main()
{

    // stores point pos
    thrust::device_vector<float> m_dBoidsPosX;
    thrust::device_vector<float> m_dBoidsPosZ;

    // stores point pos
    thrust::device_vector<float> m_dBoidsVelX;
    thrust::device_vector<float> m_dBoidsVelZ;

    // stores flocking vectors
    thrust::device_vector<float> m_dCohesionX;
    thrust::device_vector<float> m_dCohesionZ;

    thrust::device_vector<float> m_dSeperationX;
    thrust::device_vector<float> m_dSeperationZ;

    thrust::device_vector<float> m_dAlignmentX;
    thrust::device_vector<float> m_dAlignmentZ;


    //thrust::device_vector<float> d_Pos(NUM_BOIDS*3);

    // cant set size when constructing as member variable so resize here instead
    m_dBoidsPosX.resize(NUM_BOIDS);
    m_dBoidsPosZ.resize(NUM_BOIDS);

    m_dBoidsVelX.resize(NUM_BOIDS);
    m_dBoidsVelZ.resize(NUM_BOIDS);

    m_dCohesionX.resize(NUM_BOIDS);
    m_dCohesionZ.resize(NUM_BOIDS);

    m_dSeperationX.resize(NUM_BOIDS);
    m_dSeperationZ.resize(NUM_BOIDS);

    m_dAlignmentX.resize(NUM_BOIDS);
    m_dAlignmentZ.resize(NUM_BOIDS);



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
    float * m_dBoidsPosX_ptr= thrust::raw_pointer_cast(&m_dBoidsPosX[0]);
    float * m_dBoidsPosZ_ptr= thrust::raw_pointer_cast(&m_dBoidsPosZ[0]);

    float * m_dBoidsVelX_ptr= thrust::raw_pointer_cast(&m_dBoidsVelX[0]);
    float * m_dBoidsVelZ_ptr= thrust::raw_pointer_cast(&m_dBoidsVelZ[0]);

    float * m_dCohesionX_ptr= thrust::raw_pointer_cast(&m_dCohesionX[0]);
    float * m_dCohesionZ_ptr= thrust::raw_pointer_cast(&m_dCohesionZ[0]);

    float * m_dSeperationX_ptr= thrust::raw_pointer_cast(&m_dSeperationX[0]);
    float * m_dSeperationZ_ptr= thrust::raw_pointer_cast(&m_dSeperationZ[0]);

    float * m_dAlignmentX_ptr= thrust::raw_pointer_cast(&m_dAlignmentX[0]);
    float * m_dAlignmentZ_ptr= thrust::raw_pointer_cast(&m_dAlignmentZ[0]);


    //UPDATE-----------------------------------------------------------------------------

    unsigned int nThreads = 1024;
    unsigned int nBlocks = NUM_BOIDS/ nThreads + 1;

    //thrust::device_vector<unsigned int> d_numNeighbourBoids(GRID_RESOLUTION*GRID_RESOLUTION, NULL_CELL);


    //unsigned int * d_numNeighbourBoids_ptr = thrust::raw_pointer_cast(&d_numNeighbourBoids[0]);

    // for nearest neighbour
    unsigned int blockN = (NUM_BOIDS * NUM_BOIDS)/ (32*32) + 1;
    dim3 block2(32, 32); // block of (X,Y) threads
    dim3 grid2(blockN, 1); // grid blockN * blockN blocks


    for(int i = 0; i<NUM_FRAMES; i++)
    {
        // reset vectors
        thrust::fill(m_dCohesionX.begin(), m_dCohesionX.begin() + NUM_BOIDS, 0);
        thrust::fill(m_dCohesionZ.begin(), m_dCohesionZ.begin() + NUM_BOIDS, 0);

        thrust::fill(m_dSeperationX.begin(), m_dSeperationX.begin() + NUM_BOIDS, 0);
        thrust::fill(m_dSeperationZ.begin(), m_dSeperationZ.begin() + NUM_BOIDS, 0);

        thrust::fill(m_dAlignmentX.begin(), m_dAlignmentX.begin() + NUM_BOIDS, 0);
        thrust::fill(m_dAlignmentZ.begin(), m_dAlignmentZ.begin() + NUM_BOIDS, 0);




        flock_kernal<<<grid2,block2>>>(m_dCohesionX_ptr, m_dCohesionZ_ptr, m_dSeperationX_ptr, m_dSeperationZ_ptr, m_dAlignmentX_ptr, m_dAlignmentZ_ptr, m_dBoidsPosX_ptr, m_dBoidsPosZ_ptr, m_dBoidsVelX_ptr, m_dBoidsVelZ_ptr,  NUM_BOIDS);

        cudaThreadSynchronize();

        limitVel_kernal<<<nBlocks,nThreads>>>(0.02, m_dBoidsPosX_ptr, m_dBoidsPosZ_ptr, m_dBoidsVelX_ptr, m_dBoidsVelZ_ptr, NUM_BOIDS);

        cudaThreadSynchronize();

        avoidBoundaries_kernal<<<nBlocks,1024>>>(m_dBoidsPosX_ptr, m_dBoidsPosZ_ptr, m_dBoidsVelX_ptr, m_dBoidsVelZ_ptr, NUM_BOIDS);


        cudaThreadSynchronize();


        updatePos_kernal<<<nBlocks,1024>>>(m_dBoidsPosX_ptr, m_dBoidsPosZ_ptr, m_dBoidsVelX_ptr, m_dBoidsVelZ_ptr, NUM_BOIDS);

        cudaThreadSynchronize();

        dumpGeo(i, m_dBoidsPosX, m_dBoidsPosZ);


    }




}
