#include "prey_gpu.cuh"
#include "flock_gpu.cuh"

//#include "flock_kernals.cuh"


Prey_GPU::Prey_GPU(Flock_GPU *_Flock, const int _ID) : Boid_GPU(_Flock, _ID)
{

}

Prey_GPU::~Prey_GPU()
{

}


void Prey_GPU::update()
{
    //std::cout<<m_pos.x<<" \n";




    //flock();

    //avoidBoundaries();

    //printf("vel: %f,%f,%f \n",m_vel[0], m_vel[1], m_vel[2]);


//    unsigned int nThreads = 1024;
//    unsigned int nBlocks = m_Flock->getNoBoids() / nThreads + 1;

//    limitVel_kernal<<<1,1>>>(0.02, m_Flock->getBoidsPosX(), m_Flock->getBoidsPosZ(), m_Flock->getBoidsVelX(), m_Flock->getBoidsVelZ(), m_ID);
////    std::cout<<"vel: "<<m_vel[0]<<m_vel[2]<<"\n";

//     cudaThreadSynchronize();

//     avoidBoundaries_kernal<<<1,1>>>(m_Flock->getBoidsPosX(), m_Flock->getBoidsPosZ(), m_Flock->getBoidsVelX(), m_Flock->getBoidsVelZ(), m_ID);

////    std::cout<<"new vel: "<<m_vel[0]<<m_vel[2]<<"\n";


//    cudaThreadSynchronize();



//    updatePos_kernal<<<1,1>>>(m_Flock->getBoidsPosX(), m_Flock->getBoidsPosZ(), m_Flock->getBoidsVelX(), m_Flock->getBoidsVelZ(), m_ID);

//    cudaThreadSynchronize();




    //m_pos[0]+=m_vel[0];
    //m_pos[2]+=m_vel[2];



}



void Prey_GPU::avoidBoundaries()
{



    thrust::device_vector<float> desiredVel(3,0);

    if(m_pos[2] >= 2 && m_vel[2] >0)
    {
        desiredVel[0] = m_vel[0];
        desiredVel[2] = -m_vel[2];

        //printf("desired vel %f,%f,%f \n",desiredVel[0],desiredVel[1],desiredVel[2]);
        //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";

        //printf("vel: %f,%f,%f \n",m_vel[0],m_vel[1],m_vel[2]);
        m_vel[0] += steerBoid(desiredVel)[0];
        m_vel[2] += steerBoid(desiredVel)[2];
        //printf("new vel: %f,%f,%f \n",m_vel[0],m_vel[1],m_vel[2]);

        //limitVel(0.02);
        //std::cout<<" out of z bounds\n";
    }
    else if(m_pos[2] <= -2 && m_vel[2] <0)
    {
        desiredVel[0] = m_vel[0];
        desiredVel[2] = -m_vel[2];

        //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
        m_vel[0] += steerBoid(desiredVel)[0];
        m_vel[2] += steerBoid(desiredVel)[2];

        //limitVel(0.02);
        //std::cout<<" out of -z bounds\n";
    }
    else if(m_pos[0] >= 2 && m_vel[0] >0)
    {
        desiredVel[0] = -m_vel[0];
        desiredVel[2] = m_vel[2];
        //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
        m_vel[0] += steerBoid(desiredVel)[0];
        m_vel[2] += steerBoid(desiredVel)[2];

        //imitVel(0.02);
        //std::cout<<" out of x bounds\n";
    }
    else if(m_pos[0] <= -2 && m_vel[0] <0)
    {
        desiredVel[0] = -m_vel[0];
        desiredVel[2] = m_vel[2];
        //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
        m_vel[0] += steerBoid(desiredVel)[0];
        m_vel[2] += steerBoid(desiredVel)[2];

        //limitVel(0.02);
        //std::cout<<" out of -x bounds\n";
    }


    //desiredVel /=

//    std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
//    m_vel += steerBoid(desiredVel);
    //m_vel = glm::normalize(m_vel);

    //limitVel(0.02);

}


void Prey_GPU::flock()
{
        thrust::device_vector<float> steer(3,0);

        //compute the flocking component vectors
        thrust::device_vector<float> alignment(3,0);
        thrust::device_vector<float> cohesion(3,0);
        thrust::device_vector<float> separation(3,0);

        //alignment = alignBoid();
        //cohesion =cohesionBoid();
        separation =seperateBoid();


        //flocking component weights
        float alignmentWeight = 1;
        float cohesionWeight = 1;
        float separationWeight = 1.3;

        //find resulting flocking vector
        //steer[0] += (cohesion[0] * cohesionWeight) + (alignment[0] * alignmentWeight) + (separation[0] * separationWeight);
        //steer[2] += (cohesion[2] * cohesionWeight) + (alignment[2] * alignmentWeight) + (separation[2] * separationWeight);

        steer[0] += separation[0];
        steer[2] += separation[2];

        if(steer[0] != 0 && steer[2] != 0)
        {

            //steer =glm::normalize(steer);


            //steer towards flocking vector if required
            m_vel[0] += steer[0];// steerBoid(steer);
            m_vel[2] += steer[2];



        }


        if(m_vel[0] != 0  && m_vel[2] != 0)
        {
            //m_vel = glm::normalize(m_vel);


            // limit velocity
            limitVel(0.02);
        }

}

thrust::device_vector<float> Prey_GPU::alignBoid()
{
    int numberOfNeighbours = 0;
    thrust::device_vector<float> alignmentVector(3,0);

    std::vector <Prey_GPU> boidsVector = m_Flock->getBoidsVector();

    float neighbourhoodRadius = 0.3;


    // find neighbour points of current boid in desired radius
//    nearestNeighbours(0.8f,m_Flock->getHashVec()[getID()]);

    m_Flock->findNeighbours(neighbourhoodRadius,getID());



    for(int i = 0; i<m_Flock->getNoBoids(); i++)
    {
        //std::cout<<m_Flock->getNeighbours()[i]<<"\n";


        // ignore pnt_max values
        if(m_Flock->getNeighbours()[i] <= m_Flock->getNoBoids())
        {
            if(boidsVector[m_Flock->getNeighbours()[i]].getID() != getID())
            {

                //std::cout<<getNeighbourPnts()[i]<< "neighbour points of "<<getID()<<" \n";
                if(distanceToBoid(boidsVector[m_Flock->getNeighbours()[i]]) < neighbourhoodRadius)
                {

                    alignmentVector[0] += boidsVector[m_Flock->getNeighbours()[i]].m_vel[0];
                    alignmentVector[2] += boidsVector[m_Flock->getNeighbours()[i]].m_vel[2];

                    numberOfNeighbours += 1;
                }
            }
        }


    }




//    for(int i = 0; i< m_Flock->getNoBoids(); i++)
//    {
//        //only flock with other flocking boids
//        if(boidsVector[i].getID() != getID())
//        {
//            if(boidsVector[i].m_flockFlag == true)
//            {
//                if( distanceToBoid(boidsVector[i]) < 0.3)
//                {






//                    alignmentVector[0] += boidsVector[i].m_vel[0];
//                    alignmentVector[2] += boidsVector[i].m_vel[2];

//                    //printf(" updated alignment vector: %f,%f,%f \n", alignmentVector[0],alignmentVector[1],alignmentVector[2]);
//                    numberOfNeighbours += 1;
//                }
//            }
//        }
//    }

    // avoid dividing by zero
    if(numberOfNeighbours != 0)
    {




        //find average velocity of boids in the current boids neighborhood
        alignmentVector[0] /= numberOfNeighbours;
        alignmentVector[2] /= numberOfNeighbours;





        //alignmentVector =  normaliseVector(alignmentVector); // glm::normalize(alignmentVector);


    }




    return alignmentVector;

}

thrust::device_vector<float> Prey_GPU::seperateBoid()
{
    int numberOfNeighbours = 0;
    thrust::device_vector<float> seperationVector(3,0);
    std::vector <Prey_GPU> boidsVector = m_Flock->getBoidsVector();

    thrust::device_vector<float> diff(3,0);

    float neighbourhoodRadius = 0.2;


    //std::cout<<getID()<<" point id \n";

    //std::cout<<m_Flock->getHashVec()[getID()]<<" cell id \n";

    // find neighbour points of current boid
    //nearestNeighbours(neighbourhoodRadius,getID());




//    m_Flock->findNeighbours(neighbourhoodRadius,getID());



//    for(int i = 0; i<m_Flock->getNoBoids(); i++)
//    {

//        //std::cout<<m_Flock->getHashVec()[getNeighbourPnts()[i]]<< " neighbour point cell \n";

//        // ignore pnt_max values
//        if(m_Flock->getNeighbours()[i] <= m_Flock->getNoBoids())
//        {
//            if(boidsVector[m_Flock->getNeighbours()[i]].getID() != getID())
//            {

//                //std::cout<<getNeighbourPnts()[i]<< "neighbour points of "<<getID()<<" \n";
//                if(distanceToBoid(boidsVector[m_Flock->getNeighbours()[i]]) < neighbourhoodRadius)
//                {

//                //std::cout<<"seperate \n";


//                //vector from current boid to neighbor
//                diff[0] = boidsVector[m_Flock->getNeighbours()[i]].m_pos[0]-m_pos[0];
//                diff[2] = boidsVector[m_Flock->getNeighbours()[i]].m_pos[2]-m_pos[2];

//                diff = glm::normalize(diff);

//                //the closer to its neighbors the greater the seperation vector
//                seperationVector[0] += diff[0] / (distanceToBoid(boidsVector[m_Flock->getNeighbours()[i]]));
//                seperationVector[2] += diff[2] / (distanceToBoid(boidsVector[m_Flock->getNeighbours()[i]]));


//                numberOfNeighbours += 1;
//                }
//            }
//        }


//    }




    for(int i = 0; i <m_Flock->getNoBoids(); i++)
    {
        if(boidsVector[i].getID() != getID())
        {
            if(boidsVector[i].m_flockFlag == true)
            {
                if(distanceToBoid(boidsVector[i]) <0.2)
                {

                    //vector from current boid to neighbor
                    diff[0] = boidsVector[i].m_pos[0]-m_pos[0];
                    diff[2] = boidsVector[i].m_pos[2]-m_pos[2];

                    diff = normaliseVector(diff);// glm::normalize(diff);

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

        seperationVector = normaliseVector(seperationVector); //glm::normalize(seperationVector);


    }



    return seperationVector;

}

thrust::device_vector<float> Prey_GPU::cohesionBoid()
{
    int numberOfNeighbours = 0;
    thrust::device_vector<float> cohesionVector(3,0);

    std::vector <Prey_GPU> boidsVector = m_Flock->getBoidsVector();

//    std::cout<<getID()<<" point id \n";

//    std::cout<<m_Flock->getHashVec()[getID()]<<" cell id \n";

// spatial partitioning ---------------------------------------------------------------------

    // find neighbour points of current boid in desired radius
    //nearestNeighbours(1.0f,m_Flock->getHashVec()[getID()]);

    float neighbourhoodRadius = 0.4;

    m_Flock->findNeighbours(neighbourhoodRadius,getID());

    for(int i = 0; i<m_Flock->getNoBoids(); i++)
    {

        //std::cout<<m_Flock->getHashVec()[getNeighbourPnts()[i]]<< " neighbour point cell \n";

        // ignore pnt_max values
        if(m_Flock->getNeighbours()[i] <= m_Flock->getNoBoids())
        {
            if(boidsVector[m_Flock->getNeighbours()[i]].getID() != getID())
            {

                //std::cout<<getNeighbourPnts()[i]<< "neighbour points of "<<getID()<<" \n";
                if(distanceToBoid(boidsVector[m_Flock->getNeighbours()[i]]) < neighbourhoodRadius)
                {

                    cohesionVector[0] += boidsVector[m_Flock->getNeighbours()[i]].m_pos[0];
                    cohesionVector[2] += boidsVector[m_Flock->getNeighbours()[i]].m_pos[2];


                    numberOfNeighbours += 1;
                }

            }
        }


    }

    // slow code ----------------------------------------------------------------
//    for(int i = 0; i < m_Flock->getNoBoids(); i++)
//    {
//        if(boidsVector[i].getID() != getID())
//        {
//            if( boidsVector[i].m_flockFlag == true)
//            {
//                if(distanceToBoid(boidsVector[i]) < 0.4)
//                {




//                    cohesionVector[0] += boidsVector[i].m_pos[0];
//                    cohesionVector[2] += boidsVector[i].m_pos[2];


//                    numberOfNeighbours += 1;
//                }
//            }
//        }
//    }

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
        cohesionVector = normaliseVector(cohesionVector);// glm::normalize(cohesionVector);

    }

    return cohesionVector;


}

thrust::device_vector<float> Prey_GPU::steerBoid(thrust::device_vector<float> _target)
{



    thrust::device_vector<float> steerVec(3,0);

    thrust::device_vector<float> diff(3,0);


    diff[0] = _target[0] - m_vel[0];
    diff[2] = _target[2] - m_vel[2];

    //std::cout<<"steer "<<steer[0]<<steer[2]<<"\n";

    //printf("steer: %f,%f,%f \n",diff[0], diff[1], diff[2]);

    //printf("length: %f \n", vectorMagnitude(diff));

    steerVec[0] =( (diff[0]/vectorMagnitude(diff))*0.02f);
    steerVec[2] =( (diff[2]/vectorMagnitude(diff))*0.02f);



    //printf("new steer: %f,%f,%f \n",steer[0], steer[1], steer[2]);

    //std::cout<<steer[0]<<"\n";

    return steerVec;

}

float Prey_GPU::distanceToBoid(const Prey_GPU _boid)
{
    float distance = std::sqrt((m_pos[0]-_boid.m_pos[0])*(m_pos[0]-_boid.m_pos[0]) + (m_pos[2]-_boid.m_pos[2])*(m_pos[2]-_boid.m_pos[2]));

    return distance;

}

void Prey_GPU::limitVel(float _limit)
{






    if( vectorMagnitude(m_vel) > _limit)
    {

        m_vel[0] = (m_vel[0]/vectorMagnitude(m_vel))*_limit;
        m_vel[2] = (m_vel[2]/vectorMagnitude(m_vel))*_limit;

        //std::cout<<"new vel "<<m_vel[0]<<" \n";

    }
}


thrust::device_vector<float> Prey_GPU::normaliseVector(thrust::device_vector<float> _vector)
{
    thrust::device_vector<float> normalisedVector(3,0);


    normalisedVector[0] = _vector[0] / vectorMagnitude(_vector);
    normalisedVector[2] = _vector[2] / vectorMagnitude(_vector);

    return normalisedVector;



}

float Prey_GPU::vectorMagnitude(thrust::device_vector<float> _vector)
{
    float mag;

    mag = std::sqrt((_vector[0]*_vector[0]) + (_vector[2]*_vector[2]));

    return mag;
}




