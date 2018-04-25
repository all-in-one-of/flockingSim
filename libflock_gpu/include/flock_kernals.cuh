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





__global__ void avoidBoundaries_kernal(float * _posx, float * _posz, float * _velx, float * _velz, int _ID)
{
    printf("avoiding \n");

    float desiredVel[3];

    if(_posz[_ID] >= 2 && _velz[_ID] >0)
    {
        desiredVel[0] = _velx[_ID];
        desiredVel[2] = -_velz[_ID];

        //printf("desired vel %f,%f,%f \n",desiredVel[0],desiredVel[1],desiredVel[2]);
        //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";

        //printf("vel: %f,%f,%f \n",_vel[0],_vel[1],_velz[_ID]);
        _velx[_ID] = desiredVel[0]; //steerBoid(desiredVel)[0];
        _velz[_ID] = desiredVel[2]; //steerBoid(desiredVel)[2];
        //printf("new vel: %f,%f,%f \n",_vel[0],_vel[1],_velz[_ID]);

        //limitVel(0.02);
        //std::cout<<" out of z bounds\n";
    }
    else if(_posz[_ID] <= -2 && _velz[_ID] <0)
    {
        desiredVel[0] = _velx[_ID];
        desiredVel[2] = -_velz[_ID];

        //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
        _velx[_ID] = desiredVel[0]; //steerBoid(desiredVel)[0];
        _velz[_ID] = desiredVel[2]; //steerBoid(desiredVel)[2];

        //limitVel(0.02);
        //std::cout<<" out of -z bounds\n";
    }
    else if(_posx[_ID] >= 2 && _velx[_ID] >0)
    {
        desiredVel[0] = -_velx[_ID];
        desiredVel[2] = _velz[_ID];
        //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
        _velx[_ID] = desiredVel[0]; //steerBoid(desiredVel)[0];
        _velz[_ID] = desiredVel[2]; //steerBoid(desiredVel)[2];

        //imitVel(0.02);
        //std::cout<<" out of x bounds\n";
    }
    else if(_posx[_ID] <= -2 && _velx[_ID] <0)
    {
        desiredVel[0] = -_velx[_ID];
        desiredVel[2] = _velz[_ID];
        //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
        _velx[_ID] = desiredVel[0]; //steerBoid(desiredVel)[0];
        _velz[_ID] = desiredVel[2]; //steerBoid(desiredVel)[2];

        //limitVel(0.02);
        //std::cout<<" out of -x bounds\n";
    }

}

__global__ void updatePos_kernal(float * _posx, float * _posz, float * _velx, float * _velz, int _ID)
{
    printf("updating pos: %f, %f \n", _posx[_ID], _posz[_ID]);

    uint idx = blockIdx.x * blockDim.x + threadIdx.x;

    printf("id: %d \n",_ID);

    _posx[_ID] +=  _velx[_ID];
    _posz[_ID] +=  _velz[_ID];

    //printf("updating pos new: %f, %f \n", _pos[0], _pos[2]);

}

__global__ void flock_kernal()
{

}

__device__ void align_kernal()
{

}

__device__ void seperate_kernal()
{

}

__device__ void cohesion_kernal()
{

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

__device__ float distance_kernal()
{

}

__global__ void limitVel_kernal(float _limit, float * _posx, float * _posz, float * _velx, float * _velz, int _ID)
{

    float mag = sqrt((_velx[_ID]*_velz[_ID]) + (_velz[_ID]*_velz[_ID]));

    if( mag > _limit)
    {

        _velx[_ID] = (_velx[_ID]/mag)*_limit;
        _velz[_ID] = (_velz[_ID]/mag)*_limit;

        //std::cout<<"new vel "<<m_vel[0]<<" \n";

    }


}

__device__ float vectorMag_kernal()
{

}


#endif // FLOCK_KERNALS_H
