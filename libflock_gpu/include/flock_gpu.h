#ifndef FLOCK_GPU_H
#define FLOCK_GPU_H

#include "iostream"
#include <random>
#include <algorithm>
#include <vector>

#include "prey_gpu.h"

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


  //std::vector <float> m_testVec;

  //void setFlockVerts(std::vector<float> _meshVerts);
  //std::vector<float> getFlockVerts(){return m_flockVerts;}



  //std::vector<Mesh> m_flockMeshes;



//  inline void setShaderName(const std::string &_n){m_shaderName=_n;}
//  inline const std::string getShaderName()const {return m_shaderName;}


private :
    /// @brief the number of Boids
    int m_numBoids;
    /// @brief the container for the Boids
    std::vector <Prey_GPU> m_Boids;
    std::vector <int> m_hashVec;
    int m_cellOcc[36];






};


#endif // FLOCK_GPU_H
