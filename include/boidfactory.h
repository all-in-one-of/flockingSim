#ifndef BOIDFACTORY
#define BOIDFACTORY

#include "include/Boid.h"

class BoidFactory
{

public:

    Boid * createBoid(bool _Prey, int _id, Flock *_parent);



};

#endif // BOIDFACTORY

