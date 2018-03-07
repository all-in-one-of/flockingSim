#ifndef RAND_CPU_H
#define RAND_CPU_H

#include <vector>

namespace Rand_CPU {
    /// Consistent API for creating a std::vector of random floats
    int randFloats(std::vector<float>&);
}

class NearestNeighbour
{
    // flock function taking array of each voids velocity, edits velocities using flocking and returns new velocities

    // means user can just call NearestNeighbour.flock(velocities array) and have the prey flocked for them

    // will consist of nearest neighbour in parallel to detect boids in neighbourhood

};

#endif //RAND_CPU_H
