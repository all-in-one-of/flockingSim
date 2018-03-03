#include "Flock.h"
#include "boidfactory.h"

/// @brief ctor
/// @param _pos the position of the Flock
/// @param _numBoids the number of Boids to create
Flock::Flock(int _numBoids )
{
    m_numBoids=_numBoids;

    BoidFactory *b = new BoidFactory;

    m_Boids.resize(m_numBoids);

    for (int i=0; i< m_numBoids; ++i)
	{
    m_Boids.push_back(b->createBoid(true,i,this));

	}



    delete b;
}
/// @brief a method to update each of the Boids contained in the system
void Flock::update()
{
    for(auto &boid : m_Boids)
      {
        boid->update();

      }
}
/// @brief a method to draw all the Boids contained in the system
void Flock::draw()
{
    for(auto &boid : m_Boids)
      {
        boid->draw();

      }

}
