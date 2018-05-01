#ifndef PREY_H
#define PREY_H

#include "Boid.h"
#include <vector>
class Prey: public Boid
{
public:
    Prey(Flock *_Flock, const int _ID);

    ~Prey();


    /// @brief a method to update the Boid position
    void update();

    int getID() const {return m_ID;}

    glm::vec3 getVel() const {return m_vel;}

    void setVel(const glm::vec3 _vel){m_vel = _vel;}

    glm::vec3 getPos() const {return m_pos;}

    void setPos(const glm::vec3 _pos){m_pos = _pos;}

    void limitVel(const float _limit);

    glm::vec3 steerBoid(const glm::vec3 _target);



    void avoidBoundaries();


     glm::vec3 alignBoid();
     glm::vec3 seperateBoid();
     glm::vec3 cohesionBoid();

     void flock();

     bool getFlockFLag() const {return m_flockFlag;}

     float distanceToBoid(const Prey _boid);

     void nearestNeighbours(const float _neighbourhoodDist, const int cell);

     std::vector <float> getNeighbourPnts() const {return m_neighbourhoodPnts;}

private:

    /// @brief bool to determine whether the boid should flock
    bool m_flockFlag = true;

    std::vector<float> m_neighbourhoodPnts;




};

#endif // PREY_H
