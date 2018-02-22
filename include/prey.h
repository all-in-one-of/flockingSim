#ifndef PREY_H
#define PREY_H

#include "include/boid.h"


class Prey: public Boid
{
public:

    Prey();
    std::string getID(){return "prey";}

private:



};

#endif // PREY_H
