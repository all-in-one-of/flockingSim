#include "prey.h"
#include "Flock.h"

#include <ngl/Random.h>
#include <ngl/ShaderLib.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/Transformation.h>



Prey::Prey(Flock *_Flock, int _ID) : Boid(_Flock, _ID)
{

}

Prey::~Prey()
{

}

void Prey::flock()
{

}

void Prey::update()
{
    std::cout<<"update PREY \n";

    flock();

    m_pos+=m_vel;

}

void Prey::draw()
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

    prim->draw("sphere");


}

void Prey::limitVel(float _limit)
{

}

void Prey::updateRotation()
{

}




