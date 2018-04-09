#ifndef PREY_H
#define PREY_H

#include "Boid.h"
#include <vector>
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

    virtual ngl::Vec3 getPos(){return m_pos;}

    virtual void setPos(ngl::Vec3 _pos){m_pos = _pos;}

    virtual void limitVel(float _limit);

    virtual void updateRotation();

    virtual ngl::Vec3 steerBoid(ngl::Vec3 _target);

    virtual void avoidBoundaries();



    virtual int getID(){return m_ID;}

     ngl::Vec3 alignBoid();
     ngl::Vec3 seperateBoid();
     ngl::Vec3 cohesionBoid();

     void flock();

     bool getFlockFLag(){return m_flockFlag;}

     float distanceToBoid(const Prey _boid);

     void nearestNeighbours(float _neighbourhoodDist, int cell);

     std::vector <float> getNeighbourPnts(){return m_neighbourhoodPnts;}

private:

    /// @brief bool to determine whether the boid should flock
    bool m_flockFlag = true;

    std::vector<float> m_neighbourhoodPnts;




};

#endif // PREY_H
