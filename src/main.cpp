#include "attitude_check.hpp"
#include <iostream>

int main()
{
    float dt{ 100.0 };
    float gain{ 0.5 };
    float q0[] = { 1.0, 0.0, 0.0, 0.0 };

    AttitudeCheck ac(dt, gain, q0);

    std::cout << "Hello from Attitude Check" << std::endl;

    return 0;
}
