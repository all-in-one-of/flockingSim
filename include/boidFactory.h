#ifndef BOIDFACTORY
#define BOIDFACTORY

#include "Boid.h"

class Flock;

class BoidFactory
{

public:

    Boid * createBoid(bool _Prey, int _id, Flock *_parent);



};

#endif // BOIDFACTORY

