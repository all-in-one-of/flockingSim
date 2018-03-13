#ifndef BOID_GPU_H
#define BOID_GPU_H

#include <ngl/Random.h>
#include <ngl/ShaderLib.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/Transformation.h>

#include "flock_gpu.h"


#define PI = 3.14159

/// @brief ctor

Boid::Boid(Flock *_Flock, int _ID)
{
  m_ID = _ID;

  m_rotation.operator =(ngl::Vec3 {0,0,0});


  ngl::Random *rand=ngl::Random::instance();

  m_pos=rand->getRandomVec3()*3;
  m_pos.m_y = 0;

  //m_vel.operator =(ngl::Vec3{0,0,0});

  m_vel = rand->getRandomNormalizedVec3();
  m_vel.m_y = 0;
  m_vel.operator /=(10000);


  m_Flock = _Flock;
}

Boid::~Boid()
{

}

#endif // BOID_GPU_H
