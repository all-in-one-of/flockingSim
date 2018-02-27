#include "Boid.h"
#include <ngl/Random.h>
#include <ngl/ShaderLib.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/Transformation.h>
#include "Flock.h"

#define PI = 3.14159

/// @brief ctor

Boid::Boid(Flock *_Flock  )
{


  ngl::Random *rand=ngl::Random::instance();

  m_pos=rand->getRandomVec3();

  m_vel=rand->getRandomNormalizedVec3();
  m_vel.m_z = 0;
  m_vel.operator /=(10000);

  m_colour=rand->getRandomColour();
  m_lifetime=rand->randomPositiveNumber(200)*1000;
  m_currentLife=0;
  m_Flock = _Flock;
}


void Boid::updateRotation()
{
    //rotation 0 when facing in z axis
    ngl::Vec3 facing = {0,0,1};

         //only update if moving
         if(m_vel.operator !=({0,0,0}))
         {


             float mag1 = facing.length();
             float mag2 = m_vel.length() ;

             //find angle between z axis and boids velocity vector
             float steer = std::acos(facing.dot(m_vel)/(mag1*mag2));
             //math.acos(dotProduct(facing, self.velocity)/(mag1*mag2));

             //convert from radians to degrees
             steer = steer*(180/M_PI);

             //if rotation past 180 degrees must take away from 360, then update boid rotation
             if(m_vel[0]>0)
             {
                 m_rotation[1] = steer;
             }
             else
             {
                 m_rotation[1]= 360-steer;
             }
         }

}

void Boid::flock()
{
    ngl::Vec3 steer = {0,0,0};

    //compute the flocking component vectors
    ngl::Vec3 alignment;
    ngl::Vec3 cohesion;
    ngl::Vec3 separation;

    alignment.operator =(alignBoid());
    cohesion.operator =(cohesionBoid());
    separation.operator =(seperateBoid());

    //flocking component weights
    float alignmentWeight = 1;
    float cohesionWeight = 1;
    float separationWeight = 2;

    //find resulting flocking vector
    steer[0] += (cohesion[0] * cohesionWeight) + (alignment[0] * alignmentWeight) + (separation[0] * separationWeight);
    steer[2] += (cohesion[2] * cohesionWeight) + (alignment[2] * alignmentWeight) + (separation[2] * separationWeight);

    steer.normalize();

    //steer towards flocking vector
    m_vel[0] += steerBoid(steer)[0];
    m_vel[2] += steerBoid(steer)[2];

    m_vel.normalize();

//    //steer away from predator if near
//    if m_flockFlag == True && currentAgent.distanceToAgent(currentAgent.detectedPredator) < 8:

//         fleeVec = [0,0,0]

//         # vector in opossite direction to predator
//         fleeVec[0] = -(currentAgent.detectedPredator.position[0] - currentAgent.position[0])
//         fleeVec[2] = -(currentAgent.detectedPredator.position[2] - currentAgent.position[2])

//         fleeVec = normalizeVector(fleeVec)

//         predatorDist = currentAgent.distanceToAgent(currentAgent.detectedPredator)

//         # distance 0 at border and avoid weight max
//         avoidWeight = 10/(predatorDist-0.2)

//         # steer away from predator, steering force based on distance to predator
//         currentAgent.velocity[0] += self.steer(currentAgent, fleeVec)[0]*avoidWeight
//         currentAgent.velocity[2] += self.steer(currentAgent, fleeVec)[2]*avoidWeight

//         currentAgent.velocity = normalizeVector(currentAgent.velocity)


}

ngl::Vec3 Boid::alignBoid()
{

}

ngl::Vec3 Boid::seperateBoid()
{

}

ngl::Vec3 Boid::cohesionBoid()
{

}

ngl::Vec3 Boid::steerBoid(ngl::Vec3)
{

}

/// @brief a method to update the Boid position
void Boid::update()
{

    m_pos+=m_vel;
	++m_currentLife;
    updateRotation();

    //m_rotation.operator +=({0,0.01,0});

}
/// @brief a method to draw the Boid
void Boid::draw() const
{
  // get the VBO instance and draw the built in teapot
  ngl::VAOPrimitives *prim=ngl::VAOPrimitives::instance();
  ngl::Transformation trans;
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();
  shader->use(m_Flock->getShaderName());
	trans.setPosition(m_pos);
    trans.setRotation(m_rotation);

  ngl::Mat4 MV;
  ngl::Mat4 MVP;
  ngl::Mat3 normalMatrix;
  ngl::Mat4 M;
  M=trans.getMatrix();
  MV=m_Flock->getCam()->getViewMatrix()*M;
  MVP=m_Flock->getCam()->getProjectionMatrix()*MV;
  normalMatrix=MV;
  normalMatrix.inverse().transpose();
  shader->setUniform("MV",MV);
  shader->setUniform("MVP",MVP);
  shader->setUniform("normalMatrix",normalMatrix);
  shader->setUniform("M",M);

  prim->draw("cube");

}
