#include "libflock_gpu.h"
#include "flock_gpu.cuh"


libFlock::libFlock(int _numBoids)
{
    m_flock = new Flock_GPU(_numBoids);
}


void libFlock::dumpGeo(int _numFrame)
{
    m_flock->dumpGeo(_numFrame);


}


void libFlock::updateFlock()
{

    m_flock->update();


}
