#ifndef PREY_H
#define PREY_H

#include "Boid.h"
#include <vector>
class Prey: public Boid
{
public:
    Prey(Flock *_Flock, const int _ID);

    virtual ~Prey();

    /// @brief a method to update the Boid position
    virtual void update();

    virtual glm::vec3 getVel(){return m_vel;}

    virtual void setVel(glm::vec3 _vel){m_vel = _vel;}

    virtual glm::vec3 getPos(){return m_pos;}

    virtual void setPos(glm::vec3 _pos){m_pos = _pos;}

    virtual void limitVel(float _limit);

    virtual void updateRotation();

    virtual glm::vec3 steerBoid(glm::vec3 _target);

    virtual void avoidBoundaries();



    virtual int getID(){return m_ID;}

     glm::vec3 alignBoid();
     glm::vec3 seperateBoid();
     glm::vec3 cohesionBoid();

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
