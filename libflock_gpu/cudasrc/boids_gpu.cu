#include "boids_gpu.cuh"
#include <iostream>




#define PI = 3.14159

/// @brief ctor

Boid_GPU::Boid_GPU(Flock_GPU *_Flock, int _ID)
{
  m_ID = _ID;





  m_rotation.operator =(glm::vec3{0,0,0});


  m_pos={((float(rand())/RAND_MAX)-0.5)*4, 0, ((float(rand())/RAND_MAX)-0.5)*4};



  m_vel = {(float(rand())/RAND_MAX), 0, (float(rand())/RAND_MAX)};




  //m_vel = glm::normalize(m_vel);

  m_vel /= 100;

  //std::cout<<"vel "<<m_vel[0]<<" \n";


m_Flock = _Flock;


}

Boid_GPU::~Boid_GPU()
{

}
