#include "libflock_gpu.h"
#include "flock_gpu.cuh"

#include "drawScene.cuh"

void flock_gpu()
{

    //drawScene(argc,argv);


    int count = 0;


    Flock_GPU *flocknew = new Flock_GPU(20);

    //while(count < 150)
    //{
        flocknew->update();
        count ++;
    //}

//    flocknew->hash();

//    flocknew->cellOcc();

    //flocknew->findNeighbours(0.24,2);






}
