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

#include "prey_gpu.cuh"
//class Prey_GPU;
class Flock_GPU
{
public :

    /// @brief ctor
    /// @param _pos the position of the Flock
    /// @param _numBoids the number of Boids to create
    Flock_GPU(int _numBoids );
    /// @brief destructor
    ~Flock_GPU();
    /// @brief a method to update each of the Boids contained in the system
  void update();
    /// @brief a method to draw all the Boids contained in the system
  void draw();
  void createBoidsMesh();
  int getNoBoids(){return m_numBoids;}
  std::vector <Prey_GPU> getBoidsVector(){return m_Boids;}
  void hash();
  void cellOcc();

  int* getCellOcc(){return m_cellOcc;}
  std::vector <int> getHashVec(){return m_hashVec;}

  int m_gridRes = 4;

  void dumpGeo(uint _frameNumber,
               std::vector<Prey_GPU> _boids);

  int randFloats(float *&devData, const size_t n);

  // takes all points and returns neighbour point indexes
  void findNeighbours(float _neighbourhoodDist, int _currentCell);



//  float * getPos(){return m_dPos_ptr;}
    thrust::device_vector <unsigned int> getNeighbours(){return m_dneighbourPnts;}

    float* getBoidsPosX(){return m_dBoidsPosX_ptr;}
    float* getBoidsPosZ(){return m_dBoidsPosZ_ptr;}

    float* getBoidsVelX(){return m_dBoidsVelX_ptr;}
    float* getBoidsVelZ(){return m_dBoidsVelZ_ptr;}




private :
    /// @brief the number of Boids
    int m_numBoids;
    /// @brief the container for the Boids
    std::vector <Prey_GPU> m_Boids;
    std::vector <int> m_hashVec;
    int m_cellOcc[36];

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

    // stores cellOcc
    thrust::device_vector<unsigned int> m_dCellOcc;
    unsigned int * m_dCellOcc_ptr;

    // stores cellOcc
    thrust::device_vector<unsigned int> m_dHash;
    unsigned int * m_dHash_ptr;

    // stores neighbour points
    thrust::device_vector<unsigned int> m_dneighbourPnts;
    unsigned int * m_dneighbourPnts_ptr;

    int m_frame_count = 0;



};


#endif // FLOCK_GPU_H
