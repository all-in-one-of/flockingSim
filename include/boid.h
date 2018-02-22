#ifndef BOID_H
#define BOID_H

#include <iostream>
#include <fstream>


class Boid
{
public:
    Boid();
    virtual std::string getID() = 0;//{return m_id;}

protected:
    std::string m_id = "boid";


private:


};

#endif // BOID_H
