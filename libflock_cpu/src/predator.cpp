#include "predator.h"
#include "Flock.h"


//Predator::Predator(Flock *_Flock, const int _ID) : Boid(_Flock, _ID)
//{

//}


//Predator::~Predator()
//{

//}


//void Predator::update()
//{
//    //std::cout<<m_pos.x<<" \n";






//    avoidBoundaries();

//    m_pos+=m_vel;




//    updateRotation();
//}

//void Predator::draw()
//{


//        glm::mat4 MV;
//        glm::mat4 MVP;
//        glm::mat3 N;

//        // translate to new position
//        MV = glm::translate(MV, m_pos);
//        MV = glm::rotate( MV, m_rotateAngle, glm::vec3( 0.0f, 1.0f, 0.0f ) );



//        MVP = m_Flock->getScene()->getProjection() * m_Flock->getScene()->getCamera().viewMatrix() * MV;

//        N = glm::mat3( glm::inverse( glm::transpose( MV ) ) );
//        // link matrices with shader locations
//        glUniformMatrix4fv( m_Flock->getScene()->getMVPAddress(), 1, GL_FALSE, glm::value_ptr( MVP ) );
//        glUniformMatrix4fv( m_Flock->getScene()->getMVAddress(), 1, GL_FALSE, glm::value_ptr( MV ) );

//        glUniformMatrix3fv( m_Flock->getScene()->getNAddress(), 1, GL_FALSE, glm::value_ptr( N ) );


//        // draw
//        glDrawArrays( GL_TRIANGLES, 0 , ( m_Flock->getScene()->getAmountVertexData() / 3 ) );


//}

//void Predator::avoidBoundaries()
//{



//    glm::vec3 desiredVel;

//    if(m_pos.z >= 3 && m_vel.z >0)
//    {
//        desiredVel = {m_vel[0],0,-m_vel[2]};
//        //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
//        m_vel += steerBoid(desiredVel);

//        //limitVel(0.02);
//        //std::cout<<" out of z bounds\n";
//    }
//    else if(m_pos.z <= -3 && m_vel.z <0)
//    {
//        desiredVel = {m_vel[0],0,-m_vel[2]};

//        //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
//        m_vel += steerBoid(desiredVel);

//        //limitVel(0.02);
//        //std::cout<<" out of -z bounds\n";
//    }
//    else if(m_pos.x >= 3 && m_vel.x >0)
//    {
//        desiredVel = {-m_vel[0],0,m_vel[2]};
//        //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
//        m_vel += steerBoid(desiredVel);

//        //imitVel(0.02);
//        //std::cout<<" out of x bounds\n";
//    }
//    else if(m_pos.x <= -3 && m_vel.x <0)
//    {
//        desiredVel = {-m_vel[0],0,m_vel[2]};
//        //std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
//        m_vel += steerBoid(desiredVel);

//        //limitVel(0.02);
//        //std::cout<<" out of -x bounds\n";
//    }


//    //desiredVel /=

////    std::cout<<" desired vel "<<desiredVel[0]<<" "<<desiredVel[2]<<" \n";
////    m_vel += steerBoid(desiredVel);
//    //m_vel = glm::normalize(m_vel);

//    //limitVel(0.02);

//}


//void Predator::updateRotation()
//{

//    //rotation 0 when facing in z axis
//        glm::vec3 facing = {0,0,1};

//             //only update if moving
//             if(m_vel != glm::vec3{0,0,0})
//             {


//                 float mag1 = glm::length(facing);
//                 float mag2 = glm::length(m_vel);

//                 //find angle between z axis and boids velocity vector
//                 float steer = std::acos(glm::dot(facing, m_vel)/(mag1*mag2));

//                 //convert from radians to degrees
//                 //steer = steer*(180/M_PI);


//                 //std::cout<<"vel "<<m_vel[0]<<"\n";
//                 //std::cout<<"angle "<<steer<<" \n";


//                 //if rotation past 180 degrees must take away from 360, then update boid rotation
//                 if(m_vel[0]>0)
//                 {
//                     m_rotateAngle = steer;
//                     m_rotation[1] = steer;
//                 }
//                 else
//                 {
//                     m_rotateAngle = 2*M_PI -steer;
//                     m_rotation[1]= 360-steer;
//                 }
//             }

//}
