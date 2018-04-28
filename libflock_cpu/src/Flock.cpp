#include "Flock.h"
//#include "boidFactory.h"
#include <sstream>
#include <iostream>
#include <fstream>

/// @brief ctor
/// @param _pos the position of the Flock
/// @param _numBoids the number of Boids to create
Flock::Flock(int _numBoids )
{

    m_numBoids=_numBoids;

    for (int i=0; i< _numBoids; ++i)
    {

        m_Boids.push_back(Prey(this,i));

    }


}


Flock::~Flock()
{

}

void Flock::createBoidsMesh()
{
//    for(int i = 0; i< m_numBoids; i++)
//    {

//    }
}

/// @brief a method to update each of the Boids contained in the system
void Flock::update()
{



    for(int i=0; i<m_numBoids; ++i)
    {
        hash();
        cellOcc();

        m_Boids[i].update();


    }


}


void Flock::hash()
{
    // reset and recaulculate each turn
    m_hashVec.clear();

    int res = m_gridRes;
    for(int i = 0; i< m_numBoids; i++)
    {
        glm::vec3 pos = m_Boids[i].getPos();

        //std::cout<<pos.m_x<<" "<<pos.m_z<<" original \n";

        // make position between 0-1 rather then -3 - 3
        float posx = pos[0];
        float posz = pos[2];
        posx = (1.0f/6.0f)*(posx + 3);
        posz = (1.0f/6.0f)*(posz + 3);

        //std::cout<<posx<<" "<<posz<<" altered \n";


        // 2d grid
        int gridPos[2];
        gridPos[0]=floor(posx * res);
        gridPos[1]=floor(posz * res);

        //std::cout<<gridPos[0]<<" "<<gridPos[1]<<" grid \n";



        // Test to see if all of the points are inside the grid
        bool isInside = true;
        unsigned int j;
        for (j=0; j<2; ++j)
            if ((gridPos[j] < 0) || (gridPos[j] >= res)) {
                isInside = false;
            }

        // Write out the hash value if the point is within range [0,1], else write NULL_HASH
        if (isInside) {
            m_hashVec.push_back( gridPos[0] + (res * gridPos[1]));
        } else {
            m_hashVec.push_back( NULL);
        }

        //std::cout<<m_hashVec[i]<<" hash \n";


    }

}

void Flock::cellOcc()
{
    //reset cellOcc array
    for(int i = 0; i<m_gridRes*m_gridRes; i++)
    {
        m_cellOcc[i] = 0;
    }

    for(int i = 0; i<m_numBoids; i++)
    {

        //std::cout<<m_hashVec[i]<<" hash \n";
        //std::cout<<m_cellOcc[m_hashVec[i]]<<" Cells \n";
        m_cellOcc[m_hashVec[i]] +=1;


        //std::cout<<m_cellOcc[m_hashVec[i]]<<" Cells now \n";



    }

//    for(int i = 0; i<m_gridRes*m_gridRes; i++)
//    {
//        //std::cout<<m_cellOcc[i]<<" Cells \n";


//    }

    //std::cout<<"done \n";

}

/// This function was originally written by Jon Macey
void Flock::dumpGeo(uint _frameNumber)
{
            char fname[150];

            std::sprintf(fname,"geo/flock_cpu.%03d.geo",++_frameNumber);
            // we will use a stringstream as it may be more efficient
            std::stringstream ss;
            std::ofstream file;
            file.open(fname);
            if (!file.is_open())
            {
                std::cerr << "failed to Open file "<<fname<<'\n';
                exit(EXIT_FAILURE);
            }
            // write header see here http://www.sidefx.com/docs/houdini15.0/io/formats/geo
            ss << "PGEOMETRY V5\n";
            ss << "NPoints " << m_numBoids << " NPrims 1\n";
            ss << "NPointGroups 0 NPrimGroups 1\n";
            // this is hard coded but could be flexible we have 1 attrib which is Colour
            ss << "NPointAttrib 1  NVertexAttrib 0 NPrimAttrib 2 NAttrib 0\n";
            // now write out our point attrib this case Cd for diffuse colour
            ss <<"PointAttrib \n";
            // default the colour to white
            ss <<"Cd 3 float 1 1 1\n";
            // now we write out the particle data in the format
            // x y z 1 (attrib so in this case colour)
            for(unsigned int i=0; i<m_Boids.size(); ++i)
            {


                ss<<m_Boids[i].getPos().x<<" "<<m_Boids[i].getPos().y<<" "<<m_Boids[i].getPos().z << " 1 ";
                //ss<<"("<<_boids[i].cellCol.x<<" "<<_boids[i].cellCol.y<<" "<< _boids[i].cellCol.z<<")\n";
                ss<<"("<<std::abs(1)<<" "<<std::abs(1)<<" "<<std::abs(1)<<")\n";
            }

            // now write out the index values
            ss<<"PrimitiveAttrib\n";
            ss<<"generator 1 index 1 location1\n";
            ss<<"dopobject 1 index 1 /obj/AutoDopNetwork:1\n";
            ss<<"Part "<<m_Boids.size()<<" ";
            for(size_t i=0; i<m_Boids.size(); ++i)
            {
                ss<<i<<" ";
            }
            ss<<" [0	0]\n";
            ss<<"box_object1 unordered\n";
            ss<<"1 1\n";
            ss<<"beginExtra\n";
            ss<<"endExtra\n";
            // dump string stream to disk;
            file<<ss.rdbuf();
            file.close();


}
