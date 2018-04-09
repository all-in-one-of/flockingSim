#ifndef PREDATOR
#define PREDATOR

#include "Boid.h"

#include <vector>
class Predator: public Boid
{
public:
    Predator(Flock *_Flock, const int _ID);

    virtual ~Predator();

    /// @brief a method to update the Boid position
    virtual void update();

    virtual void draw();

    virtual glm::vec3 getVel(){return m_vel;}

    virtual void setVel(glm::vec3 _vel){m_vel = _vel;}

    virtual glm::vec3 getPos(){return m_pos;}

    virtual void setPos(glm::vec3 _pos){m_pos = _pos;}

    virtual void limitVel(float _limit);

    virtual void updateRotation();

    virtual glm::vec3 steerBoid(glm::vec3 _target);

    virtual void avoidBoundaries();



    virtual int getID(){return m_ID;}

    void chasePrey();


    //float distanceToBoid(const Prey _boid);

    void nearestNeighbours(float _neighbourhoodDist, int cell);

    std::vector <float> getNeighbourPnts(){return m_neighbourhoodPnts;}

private:


    std::vector<float> m_neighbourhoodPnts;




};

#endif // PREDATOR

