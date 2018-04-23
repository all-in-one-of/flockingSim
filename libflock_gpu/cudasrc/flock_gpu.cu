#include "flock_gpu.cuh"
//#include "boidFactory.h"

//#include "prey_gpu.cuh"


#include "nearestneighbour_gpu.cuh"

#include<sys/time.h>
#include <sstream>
#include <iostream>
#include <fstream>

#include "random.cuh"

/// Used to define a point not in the neighbourhood
#define NULL_PNT UINT_MAX

//__global__ void setPositions(float * posVec, float _currentPos, int _id)
//{
//    posVec[_id] = _currentPos;

//}

/// @brief ctor
/// @param _pos the position of the Flock
/// @param _numBoids the number of Boids to create
Flock_GPU::Flock_GPU(int _numBoids )
{


    m_numBoids=_numBoids;

    //thrust::device_vector<float> d_Pos(m_numBoids*3);

    m_dBoidsPosX.resize(m_numBoids);
    m_dBoidsPosZ.resize(m_numBoids);

    m_dHash.resize(m_numBoids);
    m_dCellOcc.resize(m_gridRes * m_gridRes);

    m_dneighbourPnts.resize(m_numBoids, NULL_PNT);



    m_dBoidsPosX_ptr= thrust::raw_pointer_cast(&m_dBoidsPosX[0]);
    m_dBoidsPosZ_ptr= thrust::raw_pointer_cast(&m_dBoidsPosZ[0]);


    m_dHash_ptr= thrust::raw_pointer_cast(&m_dHash[0]);
    m_dCellOcc_ptr= thrust::raw_pointer_cast(&m_dCellOcc[0]);

    m_dneighbourPnts_ptr = thrust::raw_pointer_cast(&m_dneighbourPnts[0]);






    //m_Boids.resize(m_numBoids);

    //BoidFactory *b = new BoidFactory;

    //unsigned int nThreads = 1024;
    //unsigned int nBlocks = NUM_POINTS / nThreads + 1;


    for (int i=0; i< _numBoids; ++i)
    {

        m_Boids.push_back(Prey_GPU(this,i));

        //setPositions<<<nBlocks, nThreads>>>(m_dPos_ptr, m_Boids[i].getPos().x, i);


        //Prey_GPU * prey = new Prey_GPU(this,i);





         m_dBoidsPosX[i]=m_Boids[i].getPos().x;
         m_dBoidsPosZ[i]=m_Boids[i].getPos().z;

         // make positions between 0-1 rather then -2 to 2
         m_dBoidsPosX[i] = (1.0f/4.0f)*(m_dBoidsPosX[i] + 2);
         m_dBoidsPosZ[i] = (1.0f/4.0f)*(m_dBoidsPosZ[i] + 2);

//        std::cout<<m_dPos[(3*i)]<<" "<<m_dPos[(3*i)+1]<<" "<<m_dPos[(3*i)+2]<<" \n";

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


    if(m_frame_count < 300)
    {
        //std::cout<<"dump frame \n";
        dumpGeo(m_frame_count,getBoidsVector());
        m_frame_count ++;
    }

    hash();
    cellOcc();

    for(int i=0; i<m_numBoids; ++i)
    {





        m_Boids[i].update();



        // make positions between 0-1 rather then -2 to 2
        m_dBoidsPosX[i] = (1.0f/4.0f)*(m_Boids[i].getPos().x + 2);
        m_dBoidsPosZ[i] = (1.0f/4.0f)*(m_Boids[i].getPos().z + 2);


    }


}


void Flock_GPU::findNeighbours(float _neighbourhoodDist, int _boidID)
{

    // divide by grid resolution as grid 0-1 and boids plane -2 - 2
    _neighbourhoodDist /= (2 * m_gridRes);

    // reset vector values to null_pnt
    m_dneighbourPnts.clear();
    m_dneighbourPnts.resize(m_numBoids,NULL_PNT);






        //Flock_GPU *flock = new Flock_GPU(20);

        //Prey_GPU *prey = new Prey_GPU(flock,1);



        // First thing is we'll generate a big old vector of random numbers for the purposes of
        // fleshing out our point data. This is much faster to do in one step than 3 seperate
        // steps.
        //thrust::device_vector<float> d_Rand(NUM_POINTS*3);


        //float * d_Rand_ptr = thrust::raw_pointer_cast(&d_Rand[0]);


        //randFloats(d_Rand_ptr, NUM_POINTS*3);



        // We'll store the components of the 3d vectors in separate arrays.
        // This 'structure of arrays' (SoA) approach is usually more efficient than the
        // 'array of structures' (AoS) approach.  The primary reason is that structures,
        // like Float3, don't always obey the memory coalescing rules, so they are not
        // efficiently transferred to and from memory.  Another reason to prefer SoA to
        // AoS is that we don't aways want to process all members of the structure.  For
        // example, if we only need to look at first element of the structure then it
        // is wasteful to load the entire structure from memory.  With the SoA approach,
        // we can chose which elements of the structure we wish to read.
        //thrust::device_vector<float> d_Px(d_Rand.begin(), d_Rand.begin()+NUM_POINTS);
        //thrust::device_vector<float> d_Py(d_Rand.begin()+NUM_POINTS, d_Rand.begin()+2*NUM_POINTS);
    //    thrust::device_vector<float> d_Pz(d_Rand.begin()+2*NUM_POINTS, d_Rand.end());

        // This vector will hold the grid cell occupancy (set to zero)
        //thrust::device_vector<unsigned int> d_cellOcc(GRID_RESOLUTION*GRID_RESOLUTION, 0);

        // This vector will hold our hash values, one for each point
        //thrust::device_vector<unsigned int> d_hash(NUM_POINTS);
        //thrust::copy(d_hash.begin(), d_hash.end(), std::ostream_iterator<unsigned int>(std::cout, " "));

        // Vector storing neighbour pnts idx
        //thrust::device_vector<unsigned int> d_neighbours(NUM_POINTS,NULL_PNT);

        // Holds neighbour cells
        thrust::device_vector<unsigned int> d_neighbourCells(GRID_RESOLUTION*GRID_RESOLUTION, NULL_CELL);

        // Typecast some raw pointers to the data so we can access them with CUDA functions
        //unsigned int * d_hash_ptr = thrust::raw_pointer_cast(&d_hash[0]);
        //unsigned int * d_cellOcc_ptr = thrust::raw_pointer_cast(&d_cellOcc[0]);
        //unsigned int * d_neighbours_ptr = thrust::raw_pointer_cast(&d_neighbours[0]);
        unsigned int * d_neighbourCells_ptr = thrust::raw_pointer_cast(&d_neighbourCells[0]);
        //float * d_Px_ptr = thrust::raw_pointer_cast(&d_Px[0]);
        //float * d_Py_ptr = thrust::raw_pointer_cast(&d_Py[0]);
    //   float * d_Pz_ptr = thrust::raw_pointer_cast(&d_Pz[0]);

        // The number of threads per blockshould normally be determined from your hardware, but 1024
        // is pretty standard. Remember that each block will be assigned to a single SM, with it's
        // own local memory.
        unsigned int nThreads = 1024;
        unsigned int nBlocks = NUM_POINTS / nThreads + 1;


        //dim3 threadsPerBlock(8, 8);
        //dim3 numBlocks(GRID_RESOLUTION/threadsPerBlock.x, GRID_RESOLUTION/threadsPerBlock.y);

         //int blockDim = 1024 / GRID_RESOLUTION + 1; // 9 threads per block
         dim3 block(GRID_RESOLUTION, GRID_RESOLUTION); // block of (X,Y) threads
         dim3 grid(1, 1); // grid 2x2 blocks

         // for nearest neighbour
         unsigned int blockN = NUM_POINTS/ (GRID_RESOLUTION*GRID_RESOLUTION*NUM_POINTS) + 1;
         dim3 block2(GRID_RESOLUTION*GRID_RESOLUTION, NUM_POINTS); // block of (X,Y) threads
         dim3 grid2(blockN, blockN); // grid 2x2 blocks


        struct timeval tim;
        double t1, t2;
        gettimeofday(&tim, NULL);
        t1=tim.tv_sec+(tim.tv_usec/1000000.0);

//        // The special CUDA syntax below executes our parallel function with the specified parameters
//        // using the number of blocks and threads provided.
//        pointHash<<<nBlocks, nThreads>>>(m_dHash_ptr, m_dBoidsPosX_ptr, m_dBoidsPosZ_ptr,
//                                         NUM_POINTS,
//                                         GRID_RESOLUTION);

//        // Make sure all threads have wrapped up before completing the timings
//        cudaThreadSynchronize();

//        // Now we can sort our points to ensure that points in the same grid cells occupy contiguous memory
//        thrust::sort_by_key(m_dHash.begin(), m_dHash.end(),
//                            thrust::make_zip_iterator(
//                                thrust::make_tuple( m_dBoidsPosX.begin(), m_dBoidsPosZ.begin())));

//        // Make sure all threads have wrapped up before completing the timings
//        cudaThreadSynchronize();

//        // Now we can count the number of points in each grid cell
//        countCellOccupancy<<<nBlocks, nThreads>>>(m_dCellOcc_ptr, m_dHash_ptr, m_dCellOcc.size(), m_dHash.size());

//        // Make sure all threads have wrapped up before completing the timings
//        cudaThreadSynchronize();

        neighbourhoodCells<<<grid, block>>>(d_neighbourCells_ptr,_neighbourhoodDist,GRID_RESOLUTION, m_dHash[_boidID]);

        cudaThreadSynchronize();

        // sort into order
        thrust::sort(d_neighbourCells.begin(), d_neighbourCells.end());

        //emptyCellCheck<<<nBlocks,nThreads>>>(d_neighbourCells_ptr, d_cellOcc_ptr,GRID_RESOLUTION);

        //cudaThreadSynchronize();

        // sort into order again
        //thrust::sort(d_neighbourCells.begin(), d_neighbourCells.end());

        // Finds pnt index in neighbourhood cells
        nearestNeighbourPnts<<<grid2, block2>>>(m_dneighbourPnts_ptr, d_neighbourCells_ptr, m_dHash_ptr,NUM_POINTS,GRID_RESOLUTION);

        cudaThreadSynchronize();

        // sort into order finally (PROBABLY UNNECCESERY!)
        thrust::sort(m_dneighbourPnts.begin(), m_dneighbourPnts.end());




        gettimeofday(&tim, NULL);
        t2=tim.tv_sec+(tim.tv_usec/1000000.0);
        //std::cout << "Grid sorted "<<NUM_POINTS<<" points into grid of "<<GRID_RESOLUTION*GRID_RESOLUTION*GRID_RESOLUTION<<" cells in " << t2-t1 << "s\n";

        // Only dump the debugging information if we have a manageable number of points.
//        if (NUM_POINTS <= 100) {

//            std::cout << "Boid: "<<_boidID<<"\n";
//            std::cout << "Boid Cell: "<<m_dHash[_boidID]<<"\n";
//            //thrust::copy(m_dBoidsPosX.begin(), m_dBoidsPosX.end(), std::ostream_iterator<float>(std::cout, " "));
//            //std::cout << "\n\n";
//           // thrust::copy(m_dBoidsPosZ.begin(), m_dBoidsPosZ.end(), std::ostream_iterator<float>(std::cout, " "));
//            //std::cout << "\n\n";
//            //thrust::copy(m_dHash.begin(), m_dHash.end(), std::ostream_iterator<unsigned int>(std::cout, " "));
//            //std::cout << "\n\n";
//            //thrust::copy(m_dCellOcc.begin(), m_dCellOcc.end(), std::ostream_iterator<unsigned int>(std::cout, " "));
//            //std::cout << "\n\n";
//            //thrust::copy(d_neighbourCells.begin(), d_neighbourCells.end(), std::ostream_iterator<unsigned int>(std::cout, " "));
//            //std::cout << "\n\n";
//            //thrust::copy(m_dneighbourPnts.begin(), m_dneighbourPnts.end(), std::ostream_iterator<unsigned int>(std::cout, " "));
//            std::cout << "\n\n";

//        }
        //return 0;



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

    unsigned int nThreads = 1024;
    unsigned int nBlocks = NUM_POINTS / nThreads + 1;

    // The special CUDA syntax below executes our parallel function with the specified parameters
    // using the number of blocks and threads provided.
    pointHash<<<nBlocks, nThreads>>>(m_dHash_ptr, m_dBoidsPosX_ptr, m_dBoidsPosZ_ptr,
                                     NUM_POINTS,
                                     GRID_RESOLUTION);

    // Make sure all threads have wrapped up before completing the timings
    cudaThreadSynchronize();

    // Now we can sort our points to ensure that points in the same grid cells occupy contiguous memory
    thrust::sort_by_key(m_dHash.begin(), m_dHash.end(),
                        thrust::make_zip_iterator(
                            thrust::make_tuple( m_dBoidsPosX.begin(), m_dBoidsPosZ.begin())));

    // Make sure all threads have wrapped up before completing the timings
    cudaThreadSynchronize();














//    // reset and recaulculate each turn
//    m_hashVec.clear();

//    int res = m_gridRes;
//    for(int i = 0; i< m_numBoids; i++)
//    {
//        glm::vec3 pos = m_Boids[i].getPos();

//        //std::cout<<pos.m_x<<" "<<pos.m_z<<" original \n";

//        // make position between 0-1 rather then -3 - 3
//        float posx = pos[0];
//        float posz = pos[2];
//        posx = (1.0f/8.0f)*(posx + 4);
//        posz = (1.0f/8.0f)*(posz + 4);

//        //std::cout<<posx<<" "<<posz<<" altered \n";


//        // 2d grid
//        int gridPos[2];
//        gridPos[0]=floor(posx * res);
//        gridPos[1]=floor(posz * res);

//        //std::cout<<gridPos[0]<<" "<<gridPos[1]<<" grid \n";



//        // Test to see if all of the points are inside the grid
//        bool isInside = true;
//        unsigned int j;
//        for (j=0; j<2; ++j)
//            if ((gridPos[j] < 0) || (gridPos[j] >= res)) {
//                isInside = false;
//            }

//        // Write out the hash value if the point is within range [0,1], else write NULL_HASH
//        if (isInside) {
//            m_hashVec.push_back( gridPos[0] + (res * gridPos[1]));
//        } else {
//            m_hashVec.push_back( NULL);
//        }

//        //std::cout<<m_hashVec[i]<<" hash \n";


//    }

}


void Flock_GPU::dumpGeo(uint _frameNumber, std::vector<Prey_GPU> _boids)
{
    char fname[300];

    std::sprintf(fname,"geo/flock_gpu.%03d.geo",++_frameNumber);
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
    ss << "NPoints " << getNoBoids() << " NPrims 1\n";
    ss << "NPointGroups 0 NPrimGroups 1\n";
    // this is hard coded but could be flexible we have 1 attrib which is Colour
    ss << "NPointAttrib 1  NVertexAttrib 0 NPrimAttrib 2 NAttrib 0\n";
    // now write out our point attrib this case Cd for diffuse colour
    ss <<"PointAttrib \n";
    // default the colour to white
    ss <<"Cd 3 float 1 1 1\n";
    // now we write out the particle data in the format
    // x y z 1 (attrib so in this case colour)
    for(unsigned int i=0; i<_boids.size(); ++i)
    {


        ss<<_boids[i].getPos().x<<" "<<_boids[i].getPos().y<<" "<<_boids[i].getPos().z << " 1 ";
        //ss<<"("<<_boids[i].cellCol.x<<" "<<_boids[i].cellCol.y<<" "<< _boids[i].cellCol.z<<")\n";
        ss<<"("<<std::abs(1)<<" "<<std::abs(1)<<" "<<std::abs(1)<<")\n";
    }

    // now write out the index values
    ss<<"PrimitiveAttrib\n";
    ss<<"generator 1 index 1 location1\n";
    ss<<"dopobject 1 index 1 /obj/AutoDopNetwork:1\n";
    ss<<"Part "<<_boids.size()<<" ";
    for(size_t i=0; i<_boids.size(); ++i)
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

void Flock_GPU::cellOcc()
{
    // reset vector values to 0
    m_dCellOcc.clear();
    m_dCellOcc.resize(m_gridRes*m_gridRes,0);
    unsigned int nThreads = 1024;
    unsigned int nBlocks = NUM_POINTS / nThreads + 1;

    // Now we can count the number of points in each grid cell
    countCellOccupancy<<<nBlocks, nThreads>>>(m_dCellOcc_ptr, m_dHash_ptr, m_dCellOcc.size(), m_dHash.size());

    // Make sure all threads have wrapped up before completing the timings
    cudaThreadSynchronize();




    //reset cellOcc array
//    for(int i = 0; i<m_gridRes*m_gridRes; i++)
//    {
//        m_cellOcc[i] = 0;
//    }

//    for(int i = 0; i<m_numBoids; i++)
//    {

//        //std::cout<<m_hashVec[i]<<" hash \n";
//        //std::cout<<m_cellOcc[m_hashVec[i]]<<" Cells \n";
//        m_cellOcc[m_hashVec[i]] +=1;


//        //std::cout<<m_cellOcc[m_hashVec[i]]<<" Cells now \n";



//    }

//    for(int i = 0; i<m_gridRes*m_gridRes; i++)
//    {
//        //std::cout<<m_cellOcc[i]<<" Cells \n";


//    }

    //std::cout<<"done \n";

}
