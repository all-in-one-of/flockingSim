#ifndef Boid_H_
#define Boid_H_

#include <ngl/Vec3.h>
#include <ngl/Colour.h>

// pre-declare Flock class
class Flock;

class Boid
{
public :

	/// @brief ctor
    Boid(Flock *_Flock, int _ID);

    /// @brief a method to update the Boid position
  void update();
    /// @brief a method to draw the Boid
  void draw() const;

  float distanceToBoid(const Boid _boid);

  int getID(){return m_ID;}

  ngl::Vec3 getVel(){return m_vel;}

  void setVel(ngl::Vec3 _vel){m_vel = _vel;}

  void limitVel(float _limit);

private :
    /// @brief the curent Boid' position's ID number, used to make avoid comparing with current boid in nearest neighbour
    int m_ID;
    /// @brief the curent Boid position
	ngl::Vec3 m_pos;
    /// @brief the velocity vector of the Boid
    ngl::Vec3 m_vel;

    ngl::Vec3 m_accel;
    /// @brief the rotation vector of the Boid
    ngl::Vec3 m_rotation;
    /// @brief bool to determine whether the boid should flock
    bool m_flockFlag = true;

    Flock *m_Flock;

    void updateRotation();
    void flock();

    ngl::Vec3 alignBoid();
    ngl::Vec3 seperateBoid();
    ngl::Vec3 cohesionBoid();

    ngl::Vec3 steerBoid(ngl::Vec3 _target);
};


#endif
