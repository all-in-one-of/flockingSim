#include "libflock_gpu.h"
#include "flock_gpu.cuh"

#include "drawScene.cuh"

void flock_gpu()
{

    //drawScene(argc,argv);


    int count = 0;


    Flock_GPU *flocknew = new Flock_GPU(100);

    while(count < 150)
    {
        struct timeval tim;
        double t1, t2;
        gettimeofday(&tim, NULL);
        t1=tim.tv_sec+(tim.tv_usec/1000000.0);






        flocknew->update();


        gettimeofday(&tim, NULL);
        t2=tim.tv_sec+(tim.tv_usec/1000000.0);

        std::cout << "GPU frame "<<count<<" took " << t2-t1 << "s\n";

        count ++;

    }

    std::cout<<"DONE \n";

//    flocknew->hash();

//    flocknew->cellOcc();

    //flocknew->findNeighbours(0.24,2);






}
