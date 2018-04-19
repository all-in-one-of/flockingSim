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

  int m_gridRes = 6;

  void dumpGeo(uint _frameNumber,
               std::vector<Prey_GPU> _boids);

  // takes all points and returns neighbour point indexes
  unsigned int * findNeighbours(float _neighbourhoodDist, int _boidID);



//  float * getPos(){return m_dPos_ptr;}
//  unsigned int * getNeighbours(){return m_dneighbourPnts_ptr;}




private :
    /// @brief the number of Boids
    int m_numBoids;
    /// @brief the container for the Boids
    std::vector <Prey_GPU> m_Boids;
    std::vector <int> m_hashVec;
    int m_cellOcc[36];

    // GPU -------------------------------------------------------------------------
    // stores point pos
    thrust::device_vector<float> m_dPosX;
    thrust::device_vector<float> m_dPosY;

    float * m_dPosX_ptr;
    float * m_dPosY_ptr;

    // stores neighbour points
    thrust::device_vector<unsigned int> m_dneighbourPnts;
    unsigned int * m_dneighbourPnts_ptr = thrust::raw_pointer_cast(&m_dneighbourPnts[0]);

    int m_frame_count = 0;



};


#endif // FLOCK_GPU_H
