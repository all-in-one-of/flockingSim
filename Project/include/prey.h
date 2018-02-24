#ifndef PREY_H
#define PREY_H

#include "include/Boid.h"
#include <glm.hpp>
#include <iostream>


class Prey: public Boid
{

    public:

    Prey();
    ~Prey();

    void updateRadius();
    void limitForce();
    void limitVel();

    private:

    float m_radius;

    glm::vec3 m_position;
    glm::vec3 m_velocity;
    glm::vec3 m_rotation;
    std::string m_id;

    bool m_flockFlag;


    float m_maxForce;

};

#endif // PREY_H
