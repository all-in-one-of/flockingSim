#include "include/boidFactory.h"

#include "include/prey.h"


Boid* BoidFactory::createBoid(bool _Prey, int _id, Flock *_parent)
{


    if(_Prey== true)
     {
       return new Prey(_parent,_id);
     }
     else
     {
       return new Prey(_parent,_id);
     }
}
