#ifndef BOID_H
#define BOID_H

#include <glm.hpp>
#include <iostream>


class Boid
{

    public:

    Boid();
    ~Boid();




    protected:

    float m_radius;
    glm::vec3 m_position;
    glm::vec3 m_velocity;
    glm::vec3 m_rotation;
    std::string m_id;

    float m_maxForce;

    virtual void updateRadius() = 0;
    void limitForce();
    void limitVel();



    private:


};


#endif // BOID_H
