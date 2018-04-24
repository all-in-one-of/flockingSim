#include "boids_gpu.cuh"
#include <iostream>




#define PI = 3.14159

/// @brief ctor

Boid_GPU::Boid_GPU(Flock_GPU *_Flock, int _ID)
{
  m_ID = _ID;


  // make vector 3d (cant do when constructing in header file)
  m_pos.resize(3,0);
  m_vel.resize(3,0);

  m_pos_ptr= thrust::raw_pointer_cast(&m_pos[0]);
  m_vel_ptr= thrust::raw_pointer_cast(&m_vel[0]);


  m_rotation.operator =(glm::vec3{0,0,0});


  //m_pos[0]=((float(rand())/RAND_MAX)-0.5)*4;
  //m_pos[2]=((float(rand())/RAND_MAX)-0.5)*4;




  //m_vel[0] = (float(rand())/RAND_MAX)/100;
  //m_vel[2] = (float(rand())/RAND_MAX)/100;





  //m_vel = glm::normalize(m_vel);



  //std::cout<<"vel "<<m_vel[0]<<" \n";


m_Flock = _Flock;


}

Boid_GPU::~Boid_GPU()
{

}
