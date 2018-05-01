/****************************************************************************
basic OpenGL demo modified from http://qt-project.org/doc/qt-5.0/qtgui/openglwindow.html
****************************************************************************/

#include <iostream>
#include <time.h>
#include <sys/time.h>


#include "Flock.h"

//#include "flockSim_gpu.h"
#include "libflock_gpu.h"




int main()
{

    struct timeval tim;
    double t1, t2;
    gettimeofday(&tim, NULL);
    t1=tim.tv_sec+(tim.tv_usec/1000000.0);




    libFlock gpu(1000);

    for(int i = 0; i< 150; i++)
    {

        gpu.updateFlock();

        gpu.dumpGeo(i);


    }



    gettimeofday(&tim, NULL);
    t2=tim.tv_sec+(tim.tv_usec/1000000.0);

     std::cout << "GPU took " << t2-t1 << "s\n";

    struct timeval tim_cpu;
    double t1_cpu, t2_cpu;
    gettimeofday(&tim, NULL);
    t1_cpu=tim.tv_sec+(tim.tv_usec/1000000.0);

    Flock f(100);

    for(int i = 0; i< 150; i++)
    {

        f.update();

        f.dumpGeo(i);

    }

    gettimeofday(&tim_cpu, NULL);
    t2_cpu=tim_cpu.tv_sec+(tim_cpu.tv_usec/1000000.0);

     std::cout << "CPU took " << t2_cpu-t1_cpu << "s\n";


//    // CPU Flocking
//    Flock_CPU flock1;


//    flock1.flock(argc, argv);







    return EXIT_SUCCESS;



}



