#include "libflock_gpu.h"
#include "flock_gpu.cuh"

void flock_gpu()
{


    Flock_GPU *flocknew = new Flock_GPU(20);

    flocknew->findNeighbours(0.26,2);

}
