/**
 * @file initializers.hpp
 * @brief Creates inital orientation estimates given normalized acceleration or
 *  normalized acceleration and magnetometer measurements
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include "quaternion.hpp"
#include "utilities.hpp"
#include <Eigen/Dense>

namespace init {
template<typename T>
using Vector3T = Eigen::Matrix<T, 3, 1>;

/**
 * @brief Computes a quaternion from a single normalized accerleration measurement.
 *
 * @tparam T float or double
 * @param acc Normalized 3x1 accerleration vector
 * @return quaternion::Quaternion<T>
 */
template<typename T>
inline quaternion::Quaternion<T> acc_to_quat(const Vector3T<T>& acc)
{
    T roll  = std::atan2(acc[1], acc[2]);
    T pitch = std::atan2(-acc[0],
        std::sqrt(std::pow(acc[1], 2.0) + std::pow(acc[2], 2.0)));

    return quaternion::Quaternion(utils::euler_to_quat(roll, pitch, static_cast<T>(0.0)));
}

/**
 * @brief Computes a quaternion from normalized accelerometer and magnetometer data.
 *
 * @tparam T float or double
 * @param acc Normalized 3x1 accerleration vector
 * @param mag Normalized 3x1 magnetometer vector
 * @return quaternion::Quaternion<T>
 */
template<typename T>
inline quaternion::Quaternion<T> mag_to_quat(const Vector3T<T>& acc, const Vector3T<T>& mag)
{
    T roll  = std::atan2(acc[1], acc[2]);
    T pitch = std::atan(-acc[0]/std::sqrt(std::pow(acc[1], 2.0) + std::pow(acc[2], 2.0)));

    T s_roll = std::sin(roll), c_roll = std::cos(roll);
    T yaw = std::atan2(mag[2] * s_roll - mag[1] * c_roll,
        mag[0] * std::cos(pitch) + std::sin(pitch)*(mag[1] * s_roll + mag[2] * c_roll));

    return quaternion::Quaternion(utils::euler_to_quat(roll, pitch, yaw));
}
} // End namespace init
