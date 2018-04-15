/****************************************************************************
basic OpenGL demo modified from http://qt-project.org/doc/qt-5.0/qtgui/openglwindow.html
****************************************************************************/

#include <iostream>
#include <time.h>
#include <sys/time.h>


#include "flock_cpu.h"
#include "flockSim_gpu.h"




int main(int argc, char **argv)
{

    //Flock_GPU *flock = new Flock_GPU(20);



    NearestNeighbour();
    //NearestNeighbour();



    // GPU Flocking
    //NearestNeighbour();

//    FLo
//    NearestNeighbour();







    // CPU Flocking
    Flock_CPU flock1;

    flock1.flock(argc, argv);







    return EXIT_SUCCESS;



}



