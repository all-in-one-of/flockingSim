#include "libflock_gpu.h"
#include "flock_gpu.cuh"

#include "drawScene.cuh"

void flock_gpu(int _numBoids)
{

    //drawScene(argc,argv);


    int count = 0;


    Flock_GPU *flocknew = new Flock_GPU(_numBoids);

    while(count < 150)
    {





        flocknew->update();



        count++;
    }

    std::cout<<"DONE \n";

//    flocknew->hash();

//    flocknew->cellOcc();

    //flocknew->findNeighbours(0.24,2);






}
