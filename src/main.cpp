#include <iostream>

#include "include/prey.h"



int main()
{




    Prey *deer = new Prey();

    std::cout << "Hello World!" << std::endl;

    std::cout << deer->getID() << std::endl;



    delete deer;

    return 0;
}

