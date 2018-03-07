#ifndef PREY_H
#define PREY_H

#include "Boid.h"

class Prey: public Boid
{
public:
    Prey(Flock *_Flock, int _ID);

    virtual ~Prey();

    /// @brief a method to update the Boid position
    virtual void update();

    virtual void draw();

    virtual ngl::Vec3 getVel(){return m_vel;}

    virtual void setVel(ngl::Vec3 _vel){m_vel = _vel;}

    virtual void limitVel(float _limit);

    virtual void updateRotation();

    virtual ngl::Vec3 steerBoid(ngl::Vec3 _target);



    virtual int getID(){return m_ID;}

     ngl::Vec3 alignBoid();
     ngl::Vec3 seperateBoid();
     ngl::Vec3 cohesionBoid();

     void flock();

     bool getFlockFLag(){return m_flockFlag;}

     float distanceToBoid(const Prey _boid);

private:

    /// @brief bool to determine whether the boid should flock
    bool m_flockFlag = true;



};

#endif // PREY_H
