#ifndef Flock_H_
#define Flock_H_


#include "iostream"
#include <random>
#include <algorithm>
#include <vector>

#include "prey.h"



class Flock
{
public :

	/// @brief ctor
    /// @param _pos the position of the Flock
    /// @param _numBoids the number of Boids to create
    Flock(int _numBoids);

    /// @brief destructor
    ~Flock();
    /// @brief a method to update each of the Boids contained in the system

    void update();

    int getNoBoids() const {return m_numBoids;}
    std::vector <Prey> getBoidsVector() const {return m_Boids;}

    void hash();
    void cellOcc();

    int* getCellOcc() {return m_cellOcc;}
    std::vector <int> getHashVec() const {return m_hashVec;}

    void dumpGeo(const uint _frameNumber);


    int m_gridRes = 6;

private :
    /// @brief the number of Boids
    int m_numBoids;
    /// @brief the container for the Boids
    std::vector <Prey> m_Boids;
    std::vector <int> m_hashVec;
    int m_cellOcc[36];


};


#endif

