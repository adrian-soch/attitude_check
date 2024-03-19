#pragma once

#include "quaternion.hpp"
#include "utilities.hpp"
#include <eigen3/Eigen/Dense>

namespace init {
template<typename T>
using Vector3T = Eigen::Matrix<T, 3, 1>;

template<typename T>
quaternion::Quaternion<T> acc_to_quat(const Vector3T<T>& acc)
{
    T roll  = std::atan2(acc[1], acc[acc[2]]);
    T pitch = std::atan2(-acc[0],
        std::sqrt(std::pow(acc[1], 2.0) + std::pow(acc[2], 2.0)));

    return quaternion::Quaternion(utils::euler_to_quat(roll, pitch, static_cast<T>(0.0)));
}
} // End namespace init
