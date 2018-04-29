#include "prey.h"
#include "Flock.h"




Prey::Prey(Flock *_Flock, const int _ID) : Boid(_Flock, _ID)
{

}

Prey::~Prey()
{

}


void Prey::update()
{
    //std::cout<<m_pos.x<<" \n";




    flock();

    avoidBoundaries();


    m_pos+=m_vel;




    updateRotation();
}


void Prey::avoidBoundaries()
{



    glm::vec3 desiredVel;

    if(m_pos.z >= 2 && m_vel.z >0)
    {
        desiredVel = {m_vel[0],0,-m_vel[2]};
        //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
        m_vel += steerBoid(desiredVel);

        //limitVel(0.02);
        //std::cout<<" out of z bounds\n";
    }
    else if(m_pos.z <= -2 && m_vel.z <0)
    {
        desiredVel = {m_vel[0],0,-m_vel[2]};

        //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
        m_vel += steerBoid(desiredVel);

        //limitVel(0.02);
        //std::cout<<" out of -z bounds\n";
    }
    else if(m_pos.x >= 2 && m_vel.x >0)
    {
        desiredVel = {-m_vel[0],0,m_vel[2]};
        //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
        m_vel += steerBoid(desiredVel);

        //imitVel(0.02);
        //std::cout<<" out of x bounds\n";
    }
    else if(m_pos.x <= -2 && m_vel.x <0)
    {
        desiredVel = {-m_vel[0],0,m_vel[2]};
        //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
        m_vel += steerBoid(desiredVel);

        //limitVel(0.02);
        //std::cout<<" out of -x bounds\n";
    }


    //desiredVel /=

//    std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
//    m_vel += steerBoid(desiredVel);
    //m_vel = glm::normalize(m_vel);

    //limitVel(0.02);

}


void Prey::updateRotation()
{

    //rotation 0 when facing in z axis
        glm::vec3 facing = {0,0,1};

             //only update if moving
             if(m_vel != glm::vec3{0,0,0})
             {


                 float mag1 = glm::length(facing);
                 float mag2 = glm::length(m_vel);

                 //find angle between z axis and boids velocity vector
                 float steer = std::acos(glm::dot(facing, m_vel)/(mag1*mag2));

                 //convert from radians to degrees
                 //steer = steer*(180/M_PI);


                 //std::cout<<"vel "<<m_vel[0]<<"\n";
                 //std::cout<<"angle "<<steer<<" \n";


                 //if rotation past 180 degrees must take away from 360, then update boid rotation
                 if(m_vel[0]>0)
                 {
                     m_rotateAngle = steer;
                     m_rotation[1] = steer;
                 }
                 else
                 {
                     m_rotateAngle = 2*M_PI -steer;
                     m_rotation[1]= 360-steer;
                 }
             }

}

void Prey::flock()
{
        glm::vec3 steer = {0,0,0};

        //compute the flocking component vectors
        glm::vec3 alignment = {0,0,0};
        glm::vec3 cohesion = {0,0,0};
        glm::vec3 separation = {0,0,0};

        alignment = alignBoid();
        cohesion =cohesionBoid();
        separation =seperateBoid();


        //flocking component weights
        float alignmentWeight = 1;
        float cohesionWeight = 1;
        float separationWeight = 1;

        //find resulting flocking vector
        steer[0] += (cohesion[0] * cohesionWeight) + (alignment[0] * alignmentWeight) + (separation[0] * separationWeight);
        steer[2] += (cohesion[2] * cohesionWeight) + (alignment[2] * alignmentWeight) + (separation[2] * separationWeight);

        //steer[0] += alignment[0];
        //steer[2] += alignment[2];

        if(steer != glm::vec3{0,0,0})
        {

            //steer =glm::normalize(steer);

            //steer towards flocking vector if required
            m_vel[0] +=  steerBoid(steer)[0];
            m_vel[2] +=  steerBoid(steer)[2];

        }


        if(m_vel != glm::vec3{0,0,0})
        {
            //m_vel = glm::normalize(m_vel);

            // limit velocity
            limitVel(0.02);
        }

}

glm::vec3 Prey::alignBoid()
{
    int numberOfNeighbours = 0;
    glm::vec3 alignmentVector {0,0,0};

    std::vector <Prey> boidsVector = m_Flock->getBoidsVector();


////    // find neighbour points of current boid in desired radius
//    nearestNeighbours(0.3f,m_Flock->getHashVec()[getID()]);



//    for(int i = 0; i<getNeighbourPnts().size(); i++)
//    {
//        if(boidsVector[getNeighbourPnts()[i]].getID() != getID())
//        {

//            //std::cout<<getNeighbourPnts()[i]<< "neighbour points of "<<getID()<<" \n";
//            if(distanceToBoid(boidsVector[getNeighbourPnts()[i]]) < 0.3)
//            {

//                alignmentVector[0] += boidsVector[getNeighbourPnts()[i]].m_vel[0];
//                alignmentVector[2] += boidsVector[getNeighbourPnts()[i]].m_vel[2];

//                numberOfNeighbours += 1;
//            }
//        }


//    }



    for(int i = 0; i< m_Flock->getNoBoids(); i++)
    {
        //only flock with other flocking boids
        if(boidsVector[i].getID() != getID())
        {
            if(boidsVector[i].m_flockFlag == true)
            {
                if( distanceToBoid(boidsVector[i]) < 0.15)
                {

                    alignmentVector[0] += boidsVector[i].m_vel[0];
                    alignmentVector[2] += boidsVector[i].m_vel[2];

                    numberOfNeighbours += 1;
                }
            }
        }
    }

    // avoid dividing by zero
    if(numberOfNeighbours != 0 && alignmentVector != glm::vec3{0,0,0})
    {



        //find average velocity of boids in the current boids neighborhood
        alignmentVector[0] /= numberOfNeighbours;
        alignmentVector[2] /= numberOfNeighbours;




        alignmentVector = glm::normalize(alignmentVector);
    }

    return alignmentVector;

}

glm::vec3 Prey::seperateBoid()
{
    int numberOfNeighbours = 0;
    glm::vec3 seperationVector {0,0,0};
    std::vector <Prey> boidsVector = m_Flock->getBoidsVector();

    glm::vec3 diff {0,0,0};

    //float neighbourhoodRadius = 0.2;


    //std::cout<<getID()<<" point id \n";

    //std::cout<<m_Flock->getHashVec()[getID()]<<" cell id \n";

//    // find neighbour points of current boid
//    nearestNeighbours(neighbourhoodRadius,m_Flock->getHashVec()[getID()]);


//    for(int i = 0; i<getNeighbourPnts().size(); i++)
//    {

//        //std::cout<<m_Flock->getHashVec()[getNeighbourPnts()[i]]<< " neighbour point cell \n";

//        if(boidsVector[getNeighbourPnts()[i]].getID() != getID())
//        {
//            //std::cout<<getNeighbourPnts()[i]<< "neighbour points of "<<getID()<<" \n";
//            if(distanceToBoid(boidsVector[getNeighbourPnts()[i]]) < neighbourhoodRadius)
//            {

//                //std::cout<<"seperate \n";


//                //vector from current boid to neighbor
//                diff[0] = boidsVector[getNeighbourPnts()[i]].m_pos[0]-m_pos[0];
//                diff[2] = boidsVector[getNeighbourPnts()[i]].m_pos[2]-m_pos[2];

//                diff = glm::normalize(diff);

//                //the closer to its neighbors the greater the seperation vector
//                seperationVector[0] += diff[0] / (distanceToBoid(boidsVector[getNeighbourPnts()[i]]));
//                seperationVector[2] += diff[2] / (distanceToBoid(boidsVector[getNeighbourPnts()[i]]));


//                numberOfNeighbours += 1;
//            }
//        }


//    }






    for(int i = 0; i <m_Flock->getNoBoids(); i++)
    {
        if(boidsVector[i].getID() != getID())
        {
            if(boidsVector[i].m_flockFlag == true)
            {
                if(distanceToBoid(boidsVector[i]) <0.1)
                {

                    //vector from current boid to neighbor
                    diff[0] = boidsVector[i].m_pos[0]-m_pos[0];
                    diff[2] = boidsVector[i].m_pos[2]-m_pos[2];

                    glm::normalize(diff);

                    //the closer to its neighbors the greater the seperation vector
                    seperationVector[0] += diff[0] / (distanceToBoid(boidsVector[i]));
                    seperationVector[2] += diff[2] / (distanceToBoid(boidsVector[i]));


                    numberOfNeighbours += 1;
                }
            }
        }
    }

    //avoid dividing by zero
    if(numberOfNeighbours != 0)
    {

        seperationVector[0] /= numberOfNeighbours;
        seperationVector[2] /= numberOfNeighbours;

        //run in opposite direction to average neighbor position
        seperationVector[0] *= -1;
        seperationVector[2] *= -1;

        seperationVector = glm::normalize(seperationVector);
    }



    return seperationVector;

}

glm::vec3 Prey::cohesionBoid()
{
    int numberOfNeighbours = 0;
    glm::vec3 cohesionVector {0,0,0};

    std::vector <Prey> boidsVector = m_Flock->getBoidsVector();

//    std::cout<<getID()<<" point id \n";

//    std::cout<<m_Flock->getHashVec()[getID()]<<" cell id \n";

// spatial partitioning ---------------------------------------------------------------------

//    // find neighbour points of current boid in desired radius
//    nearestNeighbours(1.0f,m_Flock->getHashVec()[getID()]);



//    for(int i = 0; i<getNeighbourPnts().size(); i++)
//    {
//        if(boidsVector[getNeighbourPnts()[i]].getID() != getID())
//        {

//            //std::cout<<getNeighbourPnts()[i]<< "neighbour points of "<<getID()<<" \n";
//            if(distanceToBoid(boidsVector[getNeighbourPnts()[i]]) < 0.4)
//            {

//                cohesionVector[0] += boidsVector[getNeighbourPnts()[i]].m_pos[0];
//                cohesionVector[2] += boidsVector[getNeighbourPnts()[i]].m_pos[2];


//                numberOfNeighbours += 1;
//            }
//        }


//    }

    // slow code ----------------------------------------------------------------
    for(int i = 0; i < m_Flock->getNoBoids(); i++)
    {
        if(boidsVector[i].getID() != getID())
        {
            if( boidsVector[i].m_flockFlag == true)
            {
                if(distanceToBoid(boidsVector[i]) < 0.2)
                {




                    cohesionVector[0] += boidsVector[i].m_pos[0];
                    cohesionVector[2] += boidsVector[i].m_pos[2];


                    numberOfNeighbours += 1;
                }
            }
        }
    }

    //avoid dividing by zero
    if(numberOfNeighbours != 0)
    {



        //find average position
        cohesionVector[0] /= numberOfNeighbours;
        cohesionVector[2] /= numberOfNeighbours;

        //find vector from agent to average position
        cohesionVector[0] = (cohesionVector[0] - m_pos[0]);
        cohesionVector[2] = (cohesionVector[2] - m_pos[2]);

        //std::cout<<cohesionVector[0]<<" "<<cohesionVector[2]<<" nomalise these\n";
        cohesionVector = glm::normalize(cohesionVector);

    }

    return cohesionVector;


}

glm::vec3 Prey::steerBoid(glm::vec3 _target)
{



    glm::vec3 steer = {0,0,0};
    steer[0] = _target[0] - m_vel[0];
    steer[2] = _target[2] - m_vel[2];

    //std::cout<<"steer "<<steer[0]<<steer[2]<<"\n";

    steer.operator =( (steer/glm::length(steer))*0.02f);

    //std::cout<<steer[0]<<"\n";

    return steer;

}

float Prey::distanceToBoid(const Prey _boid)
{
    float distance = std::sqrt((m_pos[0]-_boid.m_pos[0])*(m_pos[0]-_boid.m_pos[0]) + (m_pos[2]-_boid.m_pos[2])*(m_pos[2]-_boid.m_pos[2]));

    return distance;

}

void Prey::limitVel(float _limit)
{


    if( glm::length(m_vel) > _limit)
    {

        m_vel[0] = (m_vel[0]/glm::length(m_vel))*_limit;
        m_vel[2] = (m_vel[2]/glm::length(m_vel))*_limit;

        //std::cout<<"new vel "<<m_vel[0]<<" \n";

    }
}

void Prey::nearestNeighbours(float _neighbourhoodDist, int cell)
{





    //std::cout<<"nearest neighbour called on cell "<<cell<<" \n";

    // divide by grid resolution as grid 0-1 and boids plane -3 - 3
    _neighbourhoodDist /= (2 * m_Flock->m_gridRes);

    // the number of cells in each direction to check
    int bucketRadius = ceil(_neighbourhoodDist/(1.0/float(m_Flock->m_gridRes)));

    // Find surrounding cells
    int z = floor(float(cell/m_Flock->m_gridRes));
    int x = cell -(z*m_Flock->m_gridRes);

    int count = 0;

    int neighbourCells[m_Flock->m_gridRes*m_Flock->m_gridRes];


    for( int i = x - bucketRadius; i <= x + bucketRadius; ++i ){
        for( int j = z - bucketRadius; j <= z + bucketRadius; ++j ){
            if(i>=0 && j>=0 && i<=m_Flock->m_gridRes-1 && j<= m_Flock->m_gridRes-1)
            {
                //if((j*m_Flock->m_gridRes + i) != cell  )
                //{
                    neighbourCells[count] = (j*m_Flock->m_gridRes) + i;

                    //std::cout<<neighbourCells[count]<<" neighbour cells \n";

                    count ++;

                //}
            }

        }
    }



//    int count2;

    // Remove empty cells
    for(int i = 0; i < count; i++)
    {
        //std::cout<< neighbourCells[i]<<"\n";
        // if cell  empty
        if(m_Flock->getCellOcc()[neighbourCells[i]] == 0)
        {
            //add points to neighbourhood

            //std::cout<< neighbourCells[i]<< " deleting cell \n";
            neighbourCells[i] = -1;

//            count2++;

        }

    }


    // clear neighbour points before recalculating
    m_neighbourhoodPnts.clear();

    int count2 = 0;
    // order neighbours cells and iterate with while loop

    // find points in cells
    for(int i = 0; i < m_Flock->getNoBoids(); i++)
    {
        for(int j = 0; j<count; j++)
        {
            if(m_Flock->getHashVec()[i] == neighbourCells[j])
            {
                // add point id to list of points
                m_neighbourhoodPnts.push_back(i);

                count2++;

            }
        }

    }

}




