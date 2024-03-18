#include "attitude_check.hpp"
#include "quaternion.hpp"
#include <iostream>

int main()
{
    quaternion::Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    quaternion::Quaternion qC = q.conjugate();

    quaternion::Quaternion res = q * qC;

    std::cout << res.x() << std::endl;

    return 0;
}
