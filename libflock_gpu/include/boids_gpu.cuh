#ifndef BOIDS_GPU_H
#define BOIDS_GPU_H


#include <glm.hpp>

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>


// pre-declare Flock class
class Flock_GPU;

class Boid_GPU
{
public :

    /// @brief ctor
    Boid_GPU(Flock_GPU *_Flock, int _ID);

    virtual ~Boid_GPU();

    /// @brief a method to update the Boid position
  virtual void update() = 0;

  virtual int getID()=0;

  virtual thrust::device_vector<float> getVel()=0;

  virtual void setVel(thrust::device_vector<float> _vel) = 0;

  virtual thrust::device_vector<float> getPos()=0;

  virtual void setPos(thrust::device_vector<float> _pos) = 0;

  virtual void limitVel(float _limit) = 0;

  virtual thrust::device_vector<float> steerBoid(thrust::device_vector<float> _target) = 0;


protected :
    /// @brief the curent Boid' position's ID number, used to make avoid comparing with current boid in nearest neighbour
    int m_ID;
    /// @brief the curent Boid position
    thrust::device_vector<float> m_pos;
    float * m_pos_ptr;

    /// @brief the velocity vector of the Boid
    thrust::device_vector<float> m_vel;
    float * m_vel_ptr;

    glm::vec3 m_accel;
    /// @brief the rotation vector of the Boid
    glm::vec3 m_rotation;

    float m_rotateAngle;


    Flock_GPU *m_Flock;
};


#endif // BOIDS_GPU_H
