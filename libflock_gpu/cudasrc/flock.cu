#include "flock_gpu.cuh"
//#include "boidFactory.h"



/// @brief ctor
/// @param _pos the position of the Flock
/// @param _numBoids the number of Boids to create
Flock_GPU::Flock_GPU(int _numBoids )
{


    m_numBoids=_numBoids;

    //m_dPos.resize(m_numBoids*3);

    //m_Boids.resize(m_numBoids);

    //BoidFactory *b = new BoidFactory;

    for (int i=0; i< _numBoids; ++i)
    {

        m_Boids.push_back(Prey_GPU(this,i));


//        m_dPos[(3*i)]=m_Boids[i].getPos().x;
//        m_dPos[(3*i)+1]=m_Boids[i].getPos().y;
//        m_dPos[(3*i)+2]=m_Boids[i].getPos().z;

    }

    //m_dneighbourPnts_ptr =  thrust::raw_pointer_cast(&m_dneighbourPnts[0]);

    ///Predator(this,_numBoids+1);

    //delete b;

}

Flock_GPU::~Flock_GPU()
{

}

void Flock_GPU::createBoidsMesh()
{
//    for(int i = 0; i< m_numBoids; i++)
//    {

//    }
}

/// @brief a method to update each of the Boids contained in the system
void Flock_GPU::update()
{



    for(int i=0; i<m_numBoids; ++i)
    {






        hash();
        cellOcc();



        m_Boids[i].update();


    }


}
/// @brief a method to draw all the Boids contained in the system
void Flock_GPU::draw()
{
    for(int i=0; i<m_numBoids; ++i)
    {
        m_Boids[i].draw();
    }


}

void Flock_GPU::hash()
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

void Flock_GPU::cellOcc()
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
