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





__global__ void avoidBoundaries_kernal(float * _posx, float * _posz, float * _velx, float * _velz, int _noBoids)
{
    //printf("avoiding \n");

    uint idx = blockIdx.x * blockDim.x + threadIdx.x;

    float desiredVel[3];
    if(idx<_noBoids)
    {

        if(_posz[idx] >= 2 && _velz[idx] >0)
        {
            desiredVel[0] = _velx[idx];
            desiredVel[2] = -_velz[idx];

            //printf("desired vel %f,%f,%f \n",desiredVel[0],desiredVel[1],desiredVel[2]);
            //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";

            //printf("vel: %f,%f,%f \n",_vel[0],_vel[1],_velz[_ID]);
            _velx[idx] = desiredVel[0]; //steerBoid(desiredVel)[0];
            _velz[idx] = desiredVel[2]; //steerBoid(desiredVel)[2];
            //printf("new vel: %f,%f,%f \n",_vel[0],_vel[1],_velz[idx]);

            //limitVel(0.02);
            //std::cout<<" out of z bounds\n";
        }
        else if(_posz[idx] <= -2 && _velz[idx] <0)
        {
            desiredVel[0] = _velx[idx];
            desiredVel[2] = -_velz[idx];

            //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
            _velx[idx] = desiredVel[0]; //steerBoid(desiredVel)[0];
            _velz[idx] = desiredVel[2]; //steerBoid(desiredVel)[2];

            //limitVel(0.02);
            //std::cout<<" out of -z bounds\n";
        }
        else if(_posx[idx] >= 2 && _velx[idx] >0)
        {
            desiredVel[0] = -_velx[idx];
            desiredVel[2] = _velz[idx];
            //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
            _velx[idx] = desiredVel[0]; //steerBoid(desiredVel)[0];
            _velz[idx] = desiredVel[2]; //steerBoid(desiredVel)[2];

            //imitVel(0.02);
            //std::cout<<" out of x bounds\n";
        }
        else if(_posx[idx] <= -2 && _velx[idx] <0)
        {
            desiredVel[0] = -_velx[idx];
            desiredVel[2] = _velz[idx];
            //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
            _velx[idx] = desiredVel[0]; //steerBoid(desiredVel)[0];
            _velz[idx] = desiredVel[2]; //steerBoid(desiredVel)[2];

            //limitVel(0.02);
            //std::cout<<" out of -x bounds\n";
        }
    }

}

__global__ void updatePos_kernal(float * _posx, float * _posz, float * _velx, float * _velz, int _noBoids)
{


    uint idx = blockIdx.x * blockDim.x + threadIdx.x;



    if(idx< _noBoids)
    {
     printf("id: %d \n",idx);

    _posx[idx] +=  _velx[idx];
    _posz[idx] +=  _velz[idx];

    }

    //printf("updating pos new: %f, %f \n", _pos[0], _pos[2]);

}


__device__ float * steerBoid_kernal(float * _target, float * _current)
{
//    float steerVec[3];

//    float diff[3];


//    diff[0] = _target[0] - _current[0];
//    diff[2] = _target[2] - _current[2];

//    //std::cout<<"steer "<<steer[0]<<steer[2]<<"\n";

//    //printf("steer: %f,%f,%f \n",diff[0], diff[1], diff[2]);

//    //printf("length: %f \n", vectorMagnitude(diff));

//    steerVec[0] =( (diff[0]/vectorMagnitude(diff))*0.02f);
//    steerVec[2] =( (diff[2]/vectorMagnitude(diff))*0.02f);

}

__device__ float distance_kernal(float  _posx, float  _posz, float  _otherPosx, float  _otherPosz)
{


    float distance = sqrtf(((_posx-_otherPosx)*(_posx-_otherPosx)) + ((_posz-_otherPosz)*(_posz-_otherPosz)));


    printf("distance: %f \n", distance);

    return distance;

}

__global__ void limitVel_kernal(float _limit, float * _posx, float * _posz, float * _velx, float * _velz, int _noBoids)
{
    uint idx = blockIdx.x * blockDim.x + threadIdx.x;

    if(idx < _noBoids)
    {

        float mag = sqrt((_velx[idx]*_velz[idx]) + (_velz[idx]*_velz[idx]));

        if( mag > _limit)
        {

            _velx[idx] = (_velx[idx]/mag)*_limit;
            _velz[idx] = (_velz[idx]/mag)*_limit;

            //std::cout<<"new vel "<<m_vel[0]<<" \n";

        }
    }


}

__device__ float vectorMag_kernal(float * _vector)
{
    float mag;

    mag = sqrt((_vector[0]*_vector[0]) + (_vector[2]*_vector[2]));

    return mag;

}

__device__ void normalise_kernal(float _vector[])
{

    float normalisedVector[3];


    normalisedVector[0] = _vector[0] / vectorMag_kernal(_vector);
    normalisedVector[2] = _vector[2] / vectorMag_kernal(_vector);



}

__device__ void align_kernal()
{

}

__device__ void seperate_kernal()
{

}

__device__ void cohesion_kernal(float  _cohesionVector[], float * _posx, float * _posz, float * _velx, float * _velz, int _ID, int _numBoids)
{

    uint idx = blockIdx.x * blockDim.x + threadIdx.x;

    if(idx < _numBoids)
    {


        int numberOfNeighbours = 0;

            //for(int i = 0; i < _numBoids; i++)
            //{
                if(idx != _ID)
                {

                    if(distance_kernal(_posx[_ID], _posz[_ID], _posx[idx], _posz[idx]) < 0.4)
                    {

                        printf("position : %f, %f \n", _posx[idx], _posz[idx]);

                        _cohesionVector[0] =  _posx[idx];
                        _cohesionVector[2] =  _posz[idx];

                        printf("cohesion 1: %f, %f \n", _cohesionVector[0], _cohesionVector[2]);

                        numberOfNeighbours += 1;
                    }

                }
            //}

        printf("neighbours %d \n", numberOfNeighbours);

        //avoid dividing by zero
        if(numberOfNeighbours != 0)
        {


            //find average position
            _cohesionVector[0] /= numberOfNeighbours;
            _cohesionVector[2] /= numberOfNeighbours;

            //find vector from agent to average position
            _cohesionVector[0] = (_cohesionVector[0] - _posx[_ID]);
            _cohesionVector[2] = (_cohesionVector[2] - _posz[_ID]);

            //std::cout<<cohesionVector[0]<<" "<<cohesionVector[2]<<" nomalise these\n";
            //normalise_kernal(_cohesionVector);// glm::normalize(cohesionVector);


        }

    }

    //_cohesionVector[0]+=1;
    //_cohesionVector[2]+=1;

}

__global__ void flock_kernal(float * _posx, float * _posz, float * _velx, float * _velz, int _ID, int _numBoids)
{

    float _cohesionVector[3];

    _cohesionVector[0] = 0;
    _cohesionVector[2] = 0;

    //float *cohesion_ptr = &cohesion[0];

     __shared__ unsigned int numberOfNeighbours;

     numberOfNeighbours = 0;



    uint idx = blockIdx.x * blockDim.x + threadIdx.x;

    if(idx < _numBoids)
    {




            //for(int i = 0; i < _numBoids; i++)
            //{
                if(idx != _ID)
                {
                    printf("distance from function: %f \n",distance_kernal(_posx[_ID], _posz[_ID], _posx[idx], _posz[idx]) );

                    if(distance_kernal(_posx[_ID], _posz[_ID], _posx[idx], _posz[idx]) < 0.4)
                    {

                        printf("position : %f, %f \n", _posx[idx], _posz[idx]);


                        //atomicAdd(&(_cohesionVector[0]), _posx[idx]);
                        //atomicAdd(&(_cohesionVector[0]), _posz[idx]);

                       // _cohesionVector[0] +=  _posx[idx];
                       // _cohesionVector[2] +=  _posz[idx];

                        printf("cohesion 1: %f, %f \n", _cohesionVector[0], _cohesionVector[2]);


                        printf("Add neighbour \n");

                        atomicAdd(&numberOfNeighbours, 1);

                        //numberOfNeighbours += 1;
                    }

                }
            //}


       }

        // wait for threads to sync
        __syncthreads();

       // printf("neighbours %d \n", numberOfNeighbours);
        if(idx=0)
        {
        //avoid dividing by zero
        if(numberOfNeighbours > 0)
        {


            //find average position
            _cohesionVector[0] /= numberOfNeighbours;
            _cohesionVector[2] /= numberOfNeighbours;

            //find vector from agent to average position
            _cohesionVector[0] = ( _cohesionVector[0] - _posx[_ID]);
            _cohesionVector[2] = ( _cohesionVector[2] - _posz[_ID]);

            printf("cohesion 2: %f, %f \n", _cohesionVector[0], _cohesionVector[2]);
            //normalise_kernal(_cohesionVector);// glm::normalize(cohesionVector);


        }








    //cohesion_kernal(_cohesionVector, _posx, _posz, _velx, _velz,  _ID, _numBoids);

        printf("vel: %f, %f, %f \n",_velx[_ID],0, _velz[_ID]);

        atomicAdd(&(_velx[_ID]), _cohesionVector[0]);
        atomicAdd(&(_velz[_ID]), _cohesionVector[2]);

        printf("new vel: %f, %f, %f \n",_velx[_ID],0, _velz[_ID]);

        }
        //_velx[_ID]+= _cohesionVector[0];
        //_velz[_ID]+= _cohesionVector[2];


}





#endif // FLOCK_KERNALS_H
