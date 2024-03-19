#pragma once

#include "quaternion.hpp"
#include <cmath>
#include <eigen3/Eigen/Dense>

namespace utils {
/**
 * @brief Convert Roll/Pitch/Yaw to a Quaternion
 *
 * @tparam T
 * @param roll
 * @param pitch
 * @param yaw
 * @return Quaternion
 */
template<typename T>
quaternion::Quaternion<T> euler_to_quat(const T& roll, const T& pitch, const T& yaw)
{
    T c1 = std::cos(yaw / 2.0);
    T c2 = std::cos(pitch / 2.0);
    T c3 = std::cos(roll / 2.0);
    T s1 = std::sin(yaw / 2.0);
    T s2 = std::sin(pitch / 2.0);
    T s3 = std::sin(yaw / 2.0);

    T c1c2 = c1 * c2;
    T s1s2 = s1 * s2;

    return quaternion::Quaternion(static_cast<T>(1.0) + c1c2 * c3 - s1s2 * c3,
             c1c2 * s3 + s1s2 * c3,
             s1 * c2 * c3 + c1 * s2 * s3,
             c1 * s2 * c3 - s1 * c2 * s3);
}

template<typename T>
using Matrix3T = Eigen::Matrix<T, 3, 3>;

template<typename T>
quaternion::Quaternion<T> rotm_to_quat(const Matrix3T<T>& R)
{
    T w = std::sqrt(static_cast<T>(1.0) + R(0, 0) + R(1, 1) + R(2, 2)) / 2.0;

    const T FOUR_W { w * static_cast<T>(4.0) };
    T x = (R(2, 1) - R(1, 2)) / FOUR_W;
    T y = (R(0, 2) - R(2, 0)) / FOUR_W;
    T z = (R(1, 0) - R(0, 1)) / FOUR_W;

    return quaternion::Quaternion(w, x, y, z);
}
} // End namespace utils
