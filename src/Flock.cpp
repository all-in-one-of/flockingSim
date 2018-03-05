#include "Flock.h"
#include "boidFactory.h"
#include "prey.h"


/// @brief ctor
/// @param _pos the position of the Flock
/// @param _numBoids the number of Boids to create
Flock::Flock(int _numBoids )
{
    m_numBoids=_numBoids;

    m_Boids.resize(m_numBoids);

    BoidFactory *b = new BoidFactory;

    for (int i=0; i< _numBoids; ++i)
	{

        m_Boids.push_back(b->createBoid(true,i,this));

	}

    delete b;

}
/// @brief a method to update each of the Boids contained in the system
void Flock::update()
{
    for(int i=0; i<m_numBoids; ++i)
    {
        m_Boids[i]->update();


    }
}
/// @brief a method to draw all the Boids contained in the system
void Flock::draw()
{
    for(int i=0; i<m_numBoids; ++i)
    {
        m_Boids[i]->draw();
    }


}
