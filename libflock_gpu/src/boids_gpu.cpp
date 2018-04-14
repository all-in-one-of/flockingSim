#include "boids_gpu.h"
#include <iostream>




#define PI = 3.14159

/// @brief ctor

Boid_GPU::Boid_GPU(Flock_GPU *_Flock, int _ID)
{
  m_ID = _ID;





  m_rotation.operator =(glm::vec3{0,0,0});


  m_pos={((float(rand())/RAND_MAX)-0.5)*5, 0, ((float(rand())/RAND_MAX)-0.5)*5};


  m_vel = {(float(rand())/RAND_MAX), 0, (float(rand())/RAND_MAX)};




  //m_vel = glm::normalize(m_vel);

  m_vel /= 100;

  //std::cout<<"vel "<<m_vel[0]<<" \n";



  m_Flock = _Flock;

}

Boid_GPU::~Boid_GPU()
{

}


//void Boid::updateRotation()
//{
//    //rotation 0 when facing in z axis
//    ngl::Vec3 facing = {0,0,1};

//         //only update if moving
//         if(m_vel.operator !=({0,0,0}))
//         {


//             float mag1 = facing.length();
//             float mag2 = m_vel.length() ;

//             //find angle between z axis and boids velocity vector
//             float steer = std::acos(facing.dot(m_vel)/(mag1*mag2));
//             //math.acos(dotProduct(facing, self.velocity)/(mag1*mag2));

//             //convert from radians to degrees
//             steer = steer*(180/M_PI);

//             //if rotation past 180 degrees must take away from 360, then update boid rotation
//             if(m_vel[0]>0)
//             {
//                 m_rotation[1] = steer;
//             }
//             else
//             {
//                 m_rotation[1]= 360-steer;
//             }
//         }

//}

//void Boid::flock()
//{
//    ngl::Vec3 steer = {0,0,0};

//    //compute the flocking component vectors
//    ngl::Vec3 alignment = {0,0,0};
//    ngl::Vec3 cohesion = {0,0,0};
//    ngl::Vec3 separation = {0,0,0};

//    alignment.operator =(alignBoid());
//    cohesion.operator =(cohesionBoid());
//    separation.operator =(seperateBoid());

//    std::cout<<"align"<<alignment[0]<<alignment[2]<<"\n";
//    std::cout<<"cohesion"<<cohesion[0]<<cohesion[2]<<"\n";
//    std::cout<<"seperation"<<separation[0]<<separation[2]<<"\n";


//    //flocking component weights
//    float alignmentWeight = 1;
//    float cohesionWeight = 1;
//    float separationWeight = 1;

//    //find resulting flocking vector
//    steer[0] += (cohesion[0] * cohesionWeight) + (alignment[0] * alignmentWeight) + (separation[0] * separationWeight);
//    steer[2] += (cohesion[2] * cohesionWeight) + (alignment[2] * alignmentWeight) + (separation[2] * separationWeight);

//    //steer[0] += cohesion[0];// + separation[0];// * separationWeight;
//    //steer[2] += cohesion[2];// + separation[2];// * separationWeight;

//    //steer[0] += alignment[0];// * separationWeight;
//    //steer[2] += alignment[2];// * separationWeight;


//    if(steer.operator !=(ngl::Vec3{0,0,0}))
//    {

//        steer.normalize();

//    }




//    //steer towards flocking vector
//    m_vel[0] += steer[0];//steerBoid(steer)[0];
//    m_vel[2] += steer[2];//steerBoid(steer)[2];

//    if(m_vel.operator !=(ngl::Vec3{0,0,0}))
//    {
//        m_vel.normalize();

//        // limit velocity
//        limitVel(0.008);

//    }

////    //steer away from predator if near
////    if m_flockFlag == True && currentAgent.distanceToAgent(currentAgent.detectedPredator) < 8:

////         fleeVec = [0,0,0]

////         # vector in opossite direction to predator
////         fleeVec[0] = -(currentAgent.detectedPredator.position[0] - currentAgent.position[0])
////         fleeVec[2] = -(currentAgent.detectedPredator.position[2] - currentAgent.position[2])

////         fleeVec = normalizeVector(fleeVec)

////         predatorDist = currentAgent.distanceToAgent(currentAgent.detectedPredator)

////         # distance 0 at border and avoid weight max
////         avoidWeight = 10/(predatorDist-0.2)

////         # steer away from predator, steering force based on distance to predator
////         currentAgent.velocity[0] += self.steer(currentAgent, fleeVec)[0]*avoidWeight
////         currentAgent.velocity[2] += self.steer(currentAgent, fleeVec)[2]*avoidWeight

////         currentAgent.velocity = normalizeVector(currentAgent.velocity)


//}

//ngl::Vec3 Boid::alignBoid()
//{
//    int numberOfNeighbours = 0;
//    ngl::Vec3 alignmentVector {0,0,0};

//    std::vector <Boid> boidsVector = m_Flock->getBoidsVector();




//    for(int i = 0; i< m_Flock->getNoBoids(); i++)
//    {
//        //only flock with other flocking boids
//        if(boidsVector[i].getID() != getID())
//        {
//            if(boidsVector[i].m_flockFlag == true)
//            {
//                if( distanceToBoid(boidsVector[i]) < 0.8)
//                {

//                    alignmentVector[0] += boidsVector[i].m_vel[0];
//                    alignmentVector[2] += boidsVector[i].m_vel[2];

//                    numberOfNeighbours += 1;
//                }
//            }
//        }
//    }

//    // avoid dividing by zero
//    if(numberOfNeighbours != 0 && alignmentVector.operator !=(ngl::Vec3{0,0,0}))
//    {



//        //find average velocity of boids in the current boids neighborhood
//        alignmentVector[0] /= numberOfNeighbours;
//        alignmentVector[2] /= numberOfNeighbours;




//        alignmentVector.normalize();
//    }

//    return alignmentVector;

//}

//ngl::Vec3 Boid::seperateBoid()
//{
//    int numberOfNeighbours = 0;
//    ngl::Vec3 seperationVector {0,0,0};
//    std::vector <Boid> boidsVector = m_Flock->getBoidsVector();

//    ngl::Vec3 diff {0,0,0};



//    for(int i = 0; i <m_Flock->getNoBoids(); i++)
//    {
//        if(boidsVector[i].getID() != getID())
//        {
//            if(boidsVector[i].m_flockFlag == true)
//            {
//                if(distanceToBoid(boidsVector[i]) <0.4)
//                {

//                    //vector from current boid to neighbor
//                    diff[0] = boidsVector[i].m_pos[0]-m_pos[0];
//                    diff[2] = boidsVector[i].m_pos[2]-m_pos[2];

//                    diff.normalize();

//                    //the closer to its neighbors the greater the seperation vector
//                    seperationVector[0] += diff[0] / (distanceToBoid(boidsVector[i]));
//                    seperationVector[2] += diff[2] / (distanceToBoid(boidsVector[i]));

//                    numberOfNeighbours += 1;
//                }
//            }
//        }
//    }

//    //avoid dividing by zero
//    if(numberOfNeighbours != 0)
//    {

//        seperationVector[0] /= numberOfNeighbours;
//        seperationVector[2] /= numberOfNeighbours;

//        //run in opposite direction to average neighbor position
//        seperationVector[0] *= -1;
//        seperationVector[2] *= -1;

//        seperationVector.normalize();
//    }



//    return seperationVector;

//}

//ngl::Vec3 Boid::cohesionBoid()
//{
//    int numberOfNeighbours = 0;
//    ngl::Vec3 cohesionVector {0,0,0};

//    std::vector <Boid> boidsVector = m_Flock->getBoidsVector();

//    for(int i = 0; i < m_Flock->getNoBoids(); i++)
//    {
//        if(boidsVector[i].getID() != getID())
//        {
//            if( boidsVector[i].m_flockFlag = true)
//            {
//                if(distanceToBoid(boidsVector[i]) < 1.5)
//                {




//                    cohesionVector[0] += boidsVector[i].m_pos[0];
//                    cohesionVector[2] += boidsVector[i].m_pos[2];


//                    numberOfNeighbours += 1;
//                }
//            }
//        }
//    }

//    //avoid dividing by zero
//    if(numberOfNeighbours != 0)
//    {

//        //find average position
//        cohesionVector[0] /= numberOfNeighbours;
//        cohesionVector[2] /= numberOfNeighbours;

//        //find vector from agent to average position
//        cohesionVector[0] = (cohesionVector[0] - m_pos[0]);
//        cohesionVector[2] = (cohesionVector[2] - m_pos[2]);

//        cohesionVector.normalize();
//    }

//    return cohesionVector;


//}

//ngl::Vec3 Boid::steerBoid(ngl::Vec3 _target)
//{
//    ngl::Vec3 steer = {0,0,0};
//    steer[0] = _target[0] - m_vel[0];
//    steer[2] = _target[2] - m_vel[2];

//    //std::cout<<"steer "<<steer[0]<<"\n";

//    //steer.operator =( (steer/steer.length())*0.001);

//    //std::cout<<steer[0]<<"\n";

//    return steer;

//}

//float Boid::distanceToBoid(const Boid _boid)
//{
//    float distance = std::sqrt((m_pos[0]-_boid.m_pos[0])*(m_pos[0]-_boid.m_pos[0]) + (m_pos[2]-_boid.m_pos[2])*(m_pos[2]-_boid.m_pos[2]));

//    return distance;

//}

//void Boid::limitVel(float _limit)
//{

//    if( m_vel.length() > _limit)
//    {
//        m_vel[0] = (m_vel[0]/m_vel.length())*_limit;
//        m_vel[2] = (m_vel[2]/m_vel.length())*_limit;

//    }
//}

///// @brief a method to update the Boid position
//void Boid::update()
//{



//    flock();

//    m_pos+=m_vel;

//    //updateRotation();


//}
///// @brief a method to draw the Boid
//void Boid::draw() const
//{
//  // get the VBO instance and draw the built in teapot
//  ngl::VAOPrimitives *prim=ngl::VAOPrimitives::instance();
//  ngl::Transformation trans;
//  ngl::ShaderLib *shader=ngl::ShaderLib::instance();
//  shader->use(m_Flock->getShaderName());
//	trans.setPosition(m_pos);
//    trans.setRotation(m_rotation);

//  ngl::Mat4 MV;
//  ngl::Mat4 MVP;
//  ngl::Mat3 normalMatrix;
//  ngl::Mat4 M;
//  M=trans.getMatrix();
//  MV=m_Flock->getCam()->getViewMatrix()*M;
//  MVP=m_Flock->getCam()->getProjectionMatrix()*MV;
//  normalMatrix=MV;
//  normalMatrix.inverse().transpose();
//  shader->setUniform("MV",MV);
//  shader->setUniform("MVP",MVP);
//  shader->setUniform("normalMatrix",normalMatrix);
//  shader->setUniform("M",M);

//  prim->draw("sphere");

//}
