#ifndef Boid_H_
#define Boid_H_



#include <glm/glm.hpp>
#include <stdlib.h>

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


  virtual int getID()=0;

  virtual glm::vec3 getVel()=0;

  virtual void setVel(glm::vec3 _vel) = 0;

  virtual glm::vec3 getPos()=0;

  virtual void setPos(glm::vec3 _pos) = 0;

  virtual void limitVel(float _limit) = 0;

  virtual void updateRotation() = 0;

  virtual glm::vec3 steerBoid(glm::vec3 _target) = 0;


protected :
    /// @brief the curent Boid' position's ID number, used to make avoid comparing with current boid in nearest neighbour
    int m_ID;
    /// @brief the curent Boid position
    glm::vec3 m_pos;
    /// @brief the velocity vector of the Boid
    glm::vec3 m_vel;

    glm::vec3 m_accel;
    /// @brief the rotation vector of the Boid
    glm::vec3 m_rotation;

    float m_rotateAngle;


    Flock *m_Flock;
};


#endif
