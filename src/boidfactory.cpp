#include "include/boidfactory.h"


Boid* BoidFactory::createBoid(bool _Prey, int _id, Flock *_parent)
{


    if(_Prey== true)
     {
       return new Boid(_parent,_id);
     }
     else
     {
       return new Boid(_parent,_id);
     }
}
