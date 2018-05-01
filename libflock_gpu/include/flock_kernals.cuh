#ifndef FLOCK_KERNALS_H
#define FLOCK_KERNALS_H

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
#define NUM_BOIDS 1000

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







#endif // FLOCK_KERNALS_H
