#ifndef FLOCK_GPU_H
#define FLOCK_GPU_H

#include "iostream"
#include <random>
#include <algorithm>
#include <vector>

#include "flockSim_gpu.h"

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>



class Flock_GPU
{
public :

    /// @brief ctor
    /// @param _pos the position of the Flock
    /// @param _numBoids the number of Boids to create
    Flock_GPU(const int _numBoids );
    /// @brief destructor
    ~Flock_GPU();
    /// @brief a method to update each of the Boids contained in the system
  void update();
    /// @brief a method to draw all the Boids contained in the system
  void draw();
  void createBoidsMesh();
  int getNoBoids() const {return m_numBoids;}



  void dumpGeo(const uint _frameNumber);

  int randFloats(float *&devData, const size_t n);


    float* getBoidsPosX()const {return m_dBoidsPosX_ptr;}
    float* getBoidsPosZ()const {return m_dBoidsPosZ_ptr;}

    float* getBoidsVelX(){return m_dBoidsVelX_ptr;}
    float* getBoidsVelZ(){return m_dBoidsVelZ_ptr;}




private :
    /// @brief the number of Boids
    int m_numBoids;


    // GPU -------------------------------------------------------------------------
    // stores point pos
    thrust::device_vector<float> m_dBoidsPosX;
    thrust::device_vector<float> m_dBoidsPosZ;

    float * m_dBoidsPosX_ptr;
    float * m_dBoidsPosZ_ptr;

    // stores point pos
    thrust::device_vector<float> m_dBoidsVelX;
    thrust::device_vector<float> m_dBoidsVelZ;

    float * m_dBoidsVelX_ptr;
    float * m_dBoidsVelZ_ptr;

    // stores flocking vectors
    thrust::device_vector<float> m_dCohesionX;
    thrust::device_vector<float> m_dCohesionZ;

    float * m_dCohesionX_ptr;
    float * m_dCohesionZ_ptr;

    thrust::device_vector<float> m_dSeperationX;
    thrust::device_vector<float> m_dSeperationZ;

    float * m_dSeperationX_ptr;
    float * m_dSeperationZ_ptr;

    thrust::device_vector<float> m_dAlignmentX;
    thrust::device_vector<float> m_dAlignmentZ;

    float * m_dAlignmentX_ptr;
    float * m_dAlignmentZ_ptr;


    int m_frame_count = 0;



};


#endif // FLOCK_GPU_H
