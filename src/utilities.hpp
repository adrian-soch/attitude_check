/**
 * @file utilities.hpp
 * @brief Utilities for converting other rotation fromats into quaternions
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <cmath>

#include "quaternion.hpp"

namespace utils {
/**
 * @brief Return the norm of 3 vector components
 *
 * @param x T
 * @param y T
 * @param z T
 * @return T
 */
template<typename T>
inline T get_norm(const T& x, const T& y, const T& z)
{
    return std::sqrt(x * x + y * y + z * z);
}

/**
 * @brief Return the norm of an array components
 *
 * @tparam T
 * @tparam N
 * @param arr
 * @return T
 */
template<typename T, std::size_t N>
T get_norm(const std::array<T, N>& arr)
{
    T norm { 0.0 };

    for(T n : arr) {
        norm += n * n;
    }
    return std::sqrt(norm);
}

template<typename T, std::size_t N>
void scalar_divide(std::array<T, N>& arr, T k)
{
    for(T& n : arr) {
        n /= k;
    }
}

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
    T c_pitch = std::cos(pitch * 0.5);
    T c_roll  = std::cos(roll * 0.5);
    T s_pitch = std::sin(pitch * 0.5);
    T s_roll  = std::sin(roll * 0.5);

    if(std::fabs(yaw) <= 0.0000001) {
        return quaternion::Quaternion<T>(c_roll * c_pitch, s_roll * c_pitch, c_roll * s_pitch, -s_roll * s_pitch);
    }

    T c_yaw = std::cos(yaw * 0.5);
    T s_yaw = std::sin(yaw * 0.5);

    T c_yaw_c_pitch = c_yaw * c_pitch;
    T s_yaw_s_pitch = s_yaw * s_pitch;

    return quaternion::Quaternion<T>(c_yaw_c_pitch * c_roll + s_yaw_s_pitch * s_roll,
             c_yaw_c_pitch * s_roll - s_yaw_s_pitch * c_roll,
             s_yaw * c_pitch * s_roll + c_yaw * s_pitch * c_roll,
             s_yaw * c_pitch * c_roll - c_yaw * s_pitch * c_roll);
}

template<typename T>
using Matrix3T = std::array<std::array<T, 3>, 3>;

/**
 * @brief Convert Rotation matrix to Quaternion
 *
 * @tparam T
 * @param R rotation matrix `std::array<std::array<T, 3>, 3>`
 * @return quaternion::Quaternion<T>
 */
template<typename T>
quaternion::Quaternion<T> rotm_to_quat(const Matrix3T<T>& R)
{
    T w = std::sqrt(static_cast<T>(1.0) + R[0][0] + R[1][1] + R[2][2]) / 2.0;

    const T FOUR_W { w* static_cast<T>(4.0) };
    T x = (R[2][1] - R[1][2]) / FOUR_W;
    T y = (R[0][2] - R[2][0]) / FOUR_W;
    T z = (R[1][0] - R[0][1]) / FOUR_W;

    return quaternion::Quaternion<T>(w, x, y, z);
}
} // End namespace utils

// #endif // ifndef UTILITIES_HPP
