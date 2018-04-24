/****************************************************************************
basic OpenGL demo modified from http://qt-project.org/doc/qt-5.0/qtgui/openglwindow.html
****************************************************************************/

#include <iostream>
#include <time.h>
#include <sys/time.h>


#include "flock_cpu.h"
//#include "flockSim_gpu.h"
#include "libflock_gpu.h"




int main(int argc, char **argv)
{

    struct timeval tim;
    double t1, t2;
    gettimeofday(&tim, NULL);
    t1=tim.tv_sec+(tim.tv_usec/1000000.0);

    flock_gpu();


    gettimeofday(&tim, NULL);
    t2=tim.tv_sec+(tim.tv_usec/1000000.0);


    std::cout << "GPU Time " << t2-t1 << "s\n";









    // CPU Flocking
    Flock_CPU flock1;

    flock1.flock(argc, argv);







    return EXIT_SUCCESS;



}



