#ifndef RAND_CPU_H
#define RAND_CPU_H

#include <ngl/Random.h>
#include <ngl/ShaderLib.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/Transformation.h>

#include <vector>

namespace Rand_CPU {
    /// Consistent API for creating a std::vector of random floats
    int randFloats(std::vector<float>&);
}

namespace NearestNeighbour
{
    // flock function taking array of each voids velocity, edits velocities using flocking and returns new velocities

    // means user can just call NearestNeighbour.flock(velocities array) and have the prey flocked for them

    // will consist of nearest neighbour in parallel to detect boids in neighbourhood


    // returns array of distance to each boid
    ngl::Vec3 neighbourhood(ngl::Vec3 _pos);





}

#endif //RAND_CPU_H
