/**
 * @file utilities.hpp
 * @brief Utilities for converting other rotation fromats into quaternions
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include "quaternion.hpp"
#include <cmath>
#include <Eigen/Dense>

namespace utils {
/**
 * @brief Convert Euler angles to a Quaternion
 *  This follows the ZYX, or Yall*Pitch*Roll convention
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
    T c_pitch = cos(pitch * 0.5);
    T c_roll  = cos(roll * 0.5);
    T s_pitch = sin(pitch * 0.5);
    T s_roll  = sin(roll * 0.5);

    if(std::abs(yaw) <= 0.0000001) {
        return quaternion::Quaternion(c_roll * c_pitch, s_roll * c_pitch, c_roll * s_pitch, -s_roll * s_pitch);
    }

    T c_yaw = cos(yaw * 0.5);
    T s_yaw = sin(yaw * 0.5);

    T c_yaw_c_pitch = c_yaw * c_pitch;
    T s_yaw_s_pitch = s_yaw * s_pitch;

    return quaternion::Quaternion(c_yaw_c_pitch * c_roll + s_yaw_s_pitch * s_roll,
             c_yaw_c_pitch * s_roll - s_yaw_s_pitch * c_roll,
             s_yaw * c_pitch * s_roll + c_yaw * s_pitch * c_roll,
             s_yaw * c_pitch * c_roll - c_yaw * s_pitch * c_roll);
}

template<typename T>
using Matrix3T = Eigen::Matrix<T, 3, 3>;

/**
 * @brief Convert Rotation matrix to Quaternion
 *
 * @tparam T
 * @param R rotation matrix
 * @return quaternion::Quaternion<T>
 */
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
