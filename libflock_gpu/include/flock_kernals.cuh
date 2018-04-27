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

#define NUM_BOIDS 20



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





    float mag[NUM_BOIDS];

    if(idx < _noBoids)
    {
        printf("original vel: %f, %f id: %d \n",_velx[idx], _velz[idx], idx);

        mag[idx] = sqrtf((_velx[idx]*_velx[idx]) + (_velz[idx]*_velz[idx]));

        printf("mag: %f id: %d \n", mag[idx], idx);


        if( mag[idx] > _limit)
        {
            printf("vel: %f, %f id: %d \n",_velx[idx], _velz[idx], idx);

            _velx[idx] = (_velx[idx]/mag[idx])*_limit;
            _velz[idx] = (_velz[idx]/mag[idx])*_limit;

            printf("new vel: %f, %f id: %d \n",_velx[idx], _velz[idx], idx);


        }
    }


}

__device__ float vectorMag_kernal(float * _vector)
{
    float mag;

    mag = sqrtf((_vector[0]*_vector[0]) + (_vector[2]*_vector[2]));

    return mag;

}

__device__ void normalise_kernal(float  _v1, float  _v2, float  _v3)
{
    float _vector[3];
    _vector[0] = _v1;
    _vector[1] = _v2;
    _vector[2] =_v3;




    _v1 = _vector[0] / vectorMag_kernal(_vector);
    _v3 = _vector[2] / vectorMag_kernal(_vector);





}

__device__ void align_kernal()
{

}

__device__ void seperation_kernal(float * _seperationVectorX, float * _seperationVectorZ, float * _posx, float * _posz, float * _velx, float * _velz, int _numBoids)
{

    int const noBoids = _numBoids;

    __shared__ float _diffVectorX[NUM_BOIDS];
    __shared__ float _diffVectorZ[NUM_BOIDS];



    //float *seperation_ptr = &seperation[0];

     __shared__ unsigned int numberOfNeighbours[NUM_BOIDS];




    // current boid whos neighbours were looking for
     uint idx = blockIdx.x * blockDim.x + threadIdx.x;
    // neighbours of current boid
    uint idy = blockIdx.y * blockDim.y + threadIdx.y;

    if(idx < noBoids && idy < noBoids)
    {
         //printf("position : %f, %f, %f   id: %d\n",_posx[idx],0.0f, _posz[idx], idx);
         //printf("vel : %f, %f, %f   id: %d\n",_velx[idx],0.0f, _velz[idx], idx);

             // reset values
             numberOfNeighbours[idx] = 0;
             _diffVectorX[idx] = 0;
             _diffVectorZ[idx] = 0;

             // wait for threads to sync
             __syncthreads();


            //for(int i = 0; i < noBoids; i++)
            //{
                if(idx != idy)
                {
                    //printf("distance from function: %f \n",distance_kernal(_posx[idx], _posz[idx], _posx[idy], _posz[idy]) );

                    if(distance_kernal(_posx[idx], _posz[idx], _posx[idy], _posz[idy]) < 0.2)
                    {
                        printf("Thread x: %d, Thread y: %d \n", idx, idy );

                        printf("positionX : %f, %f positionZ : %f, %f id: %d, %d \n", _posx[idx], _posz[idx], _posx[idy], _posz[idy], idx, idy);


                        atomicAdd(&(_diffVectorX[idx]), (_posx[idy]-_posx[idx]));
                        atomicAdd(&(_diffVectorZ[idx]), (_posz[idy]-_posz[idx]));




                        printf("diff: %f, %f, %f \n", _diffVectorX[idx], 0.0f , _diffVectorZ[idx]);

                        // normalise (make atomic)
                        _diffVectorX[idx] = _diffVectorX[idx] / norm3d(_diffVectorX[idx], 0.0f, _diffVectorZ[idx]);
                        _diffVectorZ[idx] = _diffVectorZ[idx] / norm3d(_diffVectorX[idx], 0.0f, _diffVectorZ[idx]);


                        //normalise_kernal(_diffVectorX[idx], 0, _diffVectorZ[idx]);

                        printf("normalised diff: %f, %f, %f \n", _diffVectorX[idx], 0.0f , _diffVectorZ[idx]);

                        // normalise here

                        // add neighbours position to current boids part of the seperation vector
                        atomicAdd(&(_seperationVectorX[idx]), _diffVectorX[idx]);
                        atomicAdd(&(_seperationVectorZ[idx]), _diffVectorZ[idx]);



                        // add neighbours position to current boids part of the seperation vector


                       // _seperationVector[0] +=  _posx[idx];
                       // _seperationVector[2] +=  _posz[idx];

                        printf("seperation 1: %f, %f id: %d \n", _seperationVectorX[idx], _seperationVectorZ[idx], idx);


                        printf("Add neighbour \n");

                        atomicAdd(&numberOfNeighbours[idx], 1);


                        printf("neighbours %d id: %d\n", numberOfNeighbours[idx], idx);
                        //numberOfNeighbours += 1;
                    }

                }
            //}

     }




        // wait for threads to sync
        __syncthreads();

        __shared__ int currentThread;

        currentThread = 0;

        //limit to 1D
        if(idy == 0 && idx< noBoids)
        {

            printf("Thread Serial x : %d, Thread y: %d \n", idx, idy );

            //avoid dividing by zero
            if(numberOfNeighbours[idx] > 0)
            {

                printf("seperation 3: %f, %f id: %d \n", _seperationVectorX[idx], _seperationVectorZ[idx], idx);

                printf("position 2: %f, %f id: %d, \n", _posx[idx], _posz[idx], idx);

                float tmpX = _seperationVectorX[idx]/numberOfNeighbours[idx];
                float tmpZ = _seperationVectorZ[idx]/numberOfNeighbours[idx];

                printf("division: %f, %f id: %d \n", tmpX, tmpZ, idx);

                //find average position
                _seperationVectorX[idx] = tmpX;
                _seperationVectorZ[idx] = tmpZ;


                //find vector from agent to average position
                //atomicAdd(&_seperationVectorX[idx], - _posx[idx]);
                //atomicAdd(&_seperationVectorZ[idx], - _posz[idx]);


                _seperationVectorX[idx] = ( _seperationVectorX[idx] * -1);
                _seperationVectorZ[idx] = ( _seperationVectorZ[idx] * -1);

                printf("seperation 4: %f, %f id: %d \n", _seperationVectorX[idx], _seperationVectorZ[idx], idx);


                //normalise_kernal(_seperationVectorX[idx], 0, _seperationVectorZ[idx]);// glm::normalize(seperationVector);


            //printf("vel 2: %f, %f, %f   id: %d\n",_velx[idx],0.0f, _velz[idx], idx);

            //atomicAdd(&(_velx[idx]), _seperationVectorX[idx]);
            //atomicAdd(&(_velz[idx]), _seperationVectorZ[idx]);

            //_velx[idx]+= _seperationVectorX[idx];
            //_velz[idx]+= _seperationVectorZ[idx];

            //printf("new vel: %f, %f, %f id %d \n",_velx[idx],0.0f, _velz[idx], idx);

            //_velx[_ID]+= _seperationVector[0];
            //_velz[_ID]+= _seperationVector[2];}


            }

        }
}

__device__ void cohesion_kernal(float * _cohesionVectorX, float * _cohesionVectorZ, float * _posx, float * _posz, float * _velx, float * _velz, int _numBoids)
{

    int const noBoids = _numBoids;

    //__shared__ float _cohesionVectorX[NUM_BOIDS];
    //__shared__ float _cohesionVectorZ[NUM_BOIDS];



    //float *cohesion_ptr = &cohesion[0];

     __shared__ unsigned int numberOfNeighbours[NUM_BOIDS];




    // current boid whos neighbours were looking for
     uint idx = blockIdx.x * blockDim.x + threadIdx.x;
    // neighbours of current boid
    uint idy = blockIdx.y * blockDim.y + threadIdx.y;

    if(idx < noBoids && idy < noBoids)
    {
         //printf("position : %f, %f, %f   id: %d\n",_posx[idx],0.0f, _posz[idx], idx);
         //printf("vel : %f, %f, %f   id: %d\n",_velx[idx],0.0f, _velz[idx], idx);

             // reset values
             numberOfNeighbours[idx] = 0;
             _cohesionVectorX[idx] = 0;
             _cohesionVectorZ[idx] = 0;

             // wait for threads to sync
             __syncthreads();









            //for(int i = 0; i < noBoids; i++)
            //{
                if(idx != idy)
                {
                    //printf("distance from function: %f \n",distance_kernal(_posx[idx], _posz[idx], _posx[idy], _posz[idy]) );

                    if(distance_kernal(_posx[idx], _posz[idx], _posx[idy], _posz[idy]) < 0.2)
                    {
                        printf("Thread x: %d, Thread y: %d \n", idx, idy );

                        printf("position : %f, %f id: %d, %d \n", _posx[idy], _posz[idy], idx, idy);

                        // add neighbours position to current boids part of the cohesion vector
                        atomicAdd(&(_cohesionVectorX[idx]), _posx[idy]);
                        atomicAdd(&(_cohesionVectorZ[idx]), _posz[idy]);

                       // _cohesionVector[0] +=  _posx[idx];
                       // _cohesionVector[2] +=  _posz[idx];

                        printf("cohesion 1: %f, %f id: %d \n", _cohesionVectorX[idx], _cohesionVectorZ[idx], idx);


                        printf("Add neighbour \n");

                        atomicAdd(&numberOfNeighbours[idx], 1);


                        printf("neighbours %d id: %d\n", numberOfNeighbours[idx], idx);
                        //numberOfNeighbours += 1;
                    }

                }
            //}

     }




        // wait for threads to sync
        __syncthreads();

        __shared__ int currentThread;

        currentThread = 0;

        //limit to 1D
        if(idy == 0 && idx< noBoids)
        {

            printf("Thread Serial x : %d, Thread y: %d \n", idx, idy );

            //avoid dividing by zero
            if(numberOfNeighbours[idx] > 0)
            {

                printf("cohesion 3: %f, %f id: %d \n", _cohesionVectorX[idx], _cohesionVectorZ[idx], idx);

                printf("position 2: %f, %f id: %d, \n", _posx[idx], _posz[idx], idx);

                float tmpX = _cohesionVectorX[idx]/numberOfNeighbours[idx];
                float tmpZ = _cohesionVectorZ[idx]/numberOfNeighbours[idx];

                printf("division: %f, %f id: %d \n", tmpX, tmpZ, idx);

                //find average position
                _cohesionVectorX[idx] = tmpX;
                _cohesionVectorZ[idx] = tmpZ;


                //find vector from agent to average position
                //atomicAdd(&_cohesionVectorX[idx], - _posx[idx]);
                //atomicAdd(&_cohesionVectorZ[idx], - _posz[idx]);


                _cohesionVectorX[idx] = ( _cohesionVectorX[idx] - _posx[idx]);
                _cohesionVectorZ[idx] = ( _cohesionVectorZ[idx] - _posz[idx]);

                printf("cohesion 4: %f, %f id: %d \n", _cohesionVectorX[idx], _cohesionVectorZ[idx], idx);


                //normalise_kernal(_cohesionVectorX[idx], 0, _cohesionVectorZ[idx]);// glm::normalize(cohesionVector);


            //printf("vel 2: %f, %f, %f   id: %d\n",_velx[idx],0.0f, _velz[idx], idx);

            //atomicAdd(&(_velx[idx]), _cohesionVectorX[idx]);
            //atomicAdd(&(_velz[idx]), _cohesionVectorZ[idx]);

            //_velx[idx]+= _cohesionVectorX[idx];
            //_velz[idx]+= _cohesionVectorZ[idx];

            //printf("new vel: %f, %f, %f id %d \n",_velx[idx],0.0f, _velz[idx], idx);

            //_velx[_ID]+= _cohesionVector[0];
            //_velz[_ID]+= _cohesionVector[2];}


            }

        }

}

__global__ void flock_kernal(float * _cohesionVectorX, float * _cohesionVectorZ,float * _seperationVectorX, float * _seperationVectorZ, float * _posx, float * _posz, float * _velx, float * _velz, int _numBoids)
{
    uint idx = blockIdx.x * blockDim.x + threadIdx.x;
    uint idy = blockIdx.y * blockDim.y + threadIdx.y;


    if( idx <_numBoids)
    {
    // calculate cohesion
    //cohesion_kernal(_cohesionVectorX, _cohesionVectorZ, _posx, _posz, _velx, _velz, _numBoids);

    seperation_kernal(_seperationVectorX, _seperationVectorZ, _posx, _posz, _velx, _velz, _numBoids);

    // wait for threads to sync (dont add cohesion vector until calculated)
    __syncthreads();

    if(idy == 0)
    {
        //printf("cohesion output: %f, %f id: %d \n", _cohesionVectorX[idx], _cohesionVectorZ[idx], idx);

        //printf("vel 2: %f, %f, %f   id: %d\n",_velx[idx],0.0f, _velz[idx], idx);

        //_velx[idx]+= _cohesionVectorX[idx];
        //_velz[idx]+= _cohesionVectorZ[idx];

        _velx[idx]+= _seperationVectorX[idx];
        _velz[idx]+= _seperationVectorZ[idx];

        //printf("new vel: %f, %f, %f id %d \n",_velx[idx],0.0f, _velz[idx], idx);
    }

    }


//    int const noBoids = _numBoids;

//    __shared__ float _cohesionVectorX[NUM_BOIDS];
//    __shared__ float _cohesionVectorZ[NUM_BOIDS];



//    //float *cohesion_ptr = &cohesion[0];

//     __shared__ unsigned int numberOfNeighbours[NUM_BOIDS];




//    // current boid whos neighbours were looking for
//    uint idx = blockIdx.x * blockDim.x + threadIdx.x;
//    // neighbours of current boid
//    uint idy = blockIdx.y * blockDim.y + threadIdx.y;

//    if(idx < noBoids && idy < noBoids)
//    {
//         //printf("position : %f, %f, %f   id: %d\n",_posx[idx],0.0f, _posz[idx], idx);
//         //printf("vel : %f, %f, %f   id: %d\n",_velx[idx],0.0f, _velz[idx], idx);

//             // reset values
//             numberOfNeighbours[idx] = 0;
//             _cohesionVectorX[idx] = 0;
//             _cohesionVectorZ[idx] = 0;

//             // wait for threads to sync
//             __syncthreads();









//            //for(int i = 0; i < noBoids; i++)
//            //{
//                if(idx != idy)
//                {
//                    printf("distance from function: %f \n",distance_kernal(_posx[idx], _posz[idx], _posx[idy], _posz[idy]) );

//                    if(distance_kernal(_posx[idx], _posz[idx], _posx[idy], _posz[idy]) < 0.2)
//                    {
//                        printf("Thread x: %d, Thread y: %d \n", idx, idy );

//                        printf("position : %f, %f id: %d, %d \n", _posx[idy], _posz[idy], idx, idy);

//                        // add neighbours position to current boids part of the cohesion vector
//                        atomicAdd(&(_cohesionVectorX[idx]), _posx[idy]);
//                        atomicAdd(&(_cohesionVectorZ[idx]), _posz[idy]);

//                       // _cohesionVector[0] +=  _posx[idx];
//                       // _cohesionVector[2] +=  _posz[idx];

//                        printf("cohesion 1: %f, %f id: %d \n", _cohesionVectorX[idx], _cohesionVectorZ[idx], idx);


//                        printf("Add neighbour \n");

//                        atomicAdd(&numberOfNeighbours[idx], 1);


//                        printf("neighbours %d id: %d\n", numberOfNeighbours[idx], idx);
//                        //numberOfNeighbours += 1;
//                    }

//                }
//            //}

//     }




//        // wait for threads to sync
//        __syncthreads();

//        __shared__ int currentThread;

//        currentThread = 0;

//        //limit to 1D
//        if(idy == 0 && idx< noBoids)
//        {

//            printf("Thread Serial x : %d, Thread y: %d \n", idx, idy );

//            //avoid dividing by zero
//            if(numberOfNeighbours[idx] > 0)
//            {

//                printf("cohesion 3: %f, %f id: %d \n", _cohesionVectorX[idx], _cohesionVectorZ[idx], idx);

//                printf("position 2: %f, %f id: %d, \n", _posx[idx], _posz[idx], idx);

//                float tmpX = _cohesionVectorX[idx]/numberOfNeighbours[idx];
//                float tmpZ = _cohesionVectorZ[idx]/numberOfNeighbours[idx];

//                printf("division: %f, %f id: %d \n", tmpX, tmpZ, idx);

//                //find average position
//                _cohesionVectorX[idx] = tmpX;
//                _cohesionVectorZ[idx] = tmpZ;


//                //find vector from agent to average position
//                //atomicAdd(&_cohesionVectorX[idx], - _posx[idx]);
//                //atomicAdd(&_cohesionVectorZ[idx], - _posz[idx]);


//                _cohesionVectorX[idx] = ( _cohesionVectorX[idx] - _posx[idx]);
//                _cohesionVectorZ[idx] = ( _cohesionVectorZ[idx] - _posz[idx]);

//                printf("cohesion 4: %f, %f id: %d \n", _cohesionVectorX[idx], _cohesionVectorZ[idx], idx);


//                //normalise_kernal(_cohesionVectorX[idx], 0, _cohesionVectorZ[idx]);// glm::normalize(cohesionVector);


//            printf("vel 2: %f, %f, %f   id: %d\n",_velx[idx],0.0f, _velz[idx], idx);

//            atomicAdd(&(_velx[idx]), _cohesionVectorX[idx]);
//            atomicAdd(&(_velz[idx]), _cohesionVectorZ[idx]);

//            //_velx[idx]+= _cohesionVectorX[idx];
//            //_velz[idx]+= _cohesionVectorZ[idx];

//            printf("new vel: %f, %f, %f id %d \n",_velx[idx],0.0f, _velz[idx], idx);

//            //_velx[_ID]+= _cohesionVector[0];
//            //_velz[_ID]+= _cohesionVector[2];}


//            }

//        }

}







#endif // FLOCK_KERNALS_H
