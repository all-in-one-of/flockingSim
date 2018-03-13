#ifndef BOID_GPU_H
#define BOID_GPU_H

#include <ngl/Vec3.h>
#include <ngl/Colour.h>

// pre-declare Flock class
class Flock;

class Boid
{
public :

    /// @brief ctor
    Boid(Flock *_Flock, int _ID);

    virtual ~Boid();

    /// @brief a method to update the Boid position
  virtual void update() = 0;
    /// @brief a method to draw the Boid
  virtual void draw() = 0;

  virtual int getID()=0;

  virtual ngl::Vec3 getVel()=0;

  virtual void setVel(ngl::Vec3 _vel) = 0;

  virtual void limitVel(float _limit) = 0;

  virtual void updateRotation() = 0;

  virtual ngl::Vec3 steerBoid(ngl::Vec3 _target) = 0;


protected :
    /// @brief the curent Boid' position's ID number, used to make avoid comparing with current boid in nearest neighbour
    int m_ID;
    /// @brief the curent Boid position
    ngl::Vec3 m_pos;
    /// @brief the velocity vector of the Boid
    ngl::Vec3 m_vel;

    ngl::Vec3 m_accel;
    /// @brief the rotation vector of the Boid
    ngl::Vec3 m_rotation;


    Flock *m_Flock;
};

#endif // BOID_GPU_H
