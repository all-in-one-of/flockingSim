#ifndef PREY_GPU_H
#define PREY_GPU_H

#include "boids_gpu.cuh"

#include <vector>
class Prey_GPU: public Boid_GPU
{
public:
    Prey_GPU(Flock_GPU *_Flock, const int _ID);

    virtual ~Prey_GPU();

    /// @brief a method to update the Boid position
    virtual void update();

    virtual void draw();

    virtual thrust::device_vector<float> getVel(){return m_vel;}

    virtual void setVel(thrust::device_vector<float> _vel){m_vel = _vel;}

    virtual thrust::device_vector<float> getPos(){return m_pos;}

    virtual void setPos(thrust::device_vector<float> _pos){m_pos = _pos;}

    virtual void limitVel(float _limit);

    virtual void updateRotation();

    virtual thrust::device_vector<float> steerBoid(thrust::device_vector<float> _target);

    virtual void avoidBoundaries();



    virtual int getID(){return m_ID;}

     thrust::device_vector<float> alignBoid();
     thrust::device_vector<float> seperateBoid();
     thrust::device_vector<float> cohesionBoid();

     void flock();

     bool getFlockFLag(){return m_flockFlag;}

     float distanceToBoid(const Prey_GPU _boid);

     //void nearestNeighbours(float _neighbourhoodDist, int cell);

     std::vector <float> getNeighbourPnts(){return m_neighbourhoodPnts;}

     thrust::device_vector<float> normaliseVector(thrust::device_vector<float> _vector);

     float vectorMagnitude(thrust::device_vector<float> _vector);

private:

    /// @brief bool to determine whether the boid should flock
    bool m_flockFlag = true;

    std::vector<float> m_neighbourhoodPnts;




};

#endif // PREY_GPU_H
