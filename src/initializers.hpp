/**
 * @file initializers.hpp
 * @brief Creates inital orientation estimates given normalized acceleration or
 *  normalized acceleration and magnetometer measurements
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef INITIALIZERS_HPP
#define INITIALIZERS_HPP

#include <cmath>

#include "quaternion.hpp"
#include "utilities.hpp"

namespace initializers {
/**
 * @brief Computes a quaternion from a single accerleration measurement.
 *
 * @tparam T
 * @param ax
 * @param ay
 * @param az
 * @return quaternion::Quaternion<T>
 */
template<typename T>
inline quaternion::Quaternion<T> acc_to_quat(T ax, T ay, T az)
{
    T norm = utils::norm(ax, ay, az);
    ax /= norm;
    ay /= norm;
    az /= norm;

    T roll  = std::atan2(ay, az);
    T pitch = std::atan2(-ax,
        std::sqrt(ay*ay + az*az));

    return quaternion::Quaternion<T>(utils::euler_to_quat(roll, pitch, static_cast<T>(0.0)));
}

/**
 * @brief Computes a quaternion from accelerometer and magnetometer data.
 *
 * @tparam T
 * @param ax
 * @param ay
 * @param az
 * @param mx
 * @param my
 * @param mz
 * @return quaternion::Quaternion<T>
 */
template<typename T>
inline quaternion::Quaternion<T> mag_to_quat(T ax, T ay, T az, T mx, T my, T mz)
{
    T a_norm = utils::norm(ax, ay, az);
    ax /= a_norm;
    ay /= a_norm;
    az /= a_norm;

    T m_norm = utils::norm(mx, my, mz);
    mx /= m_norm;
    my /= m_norm;
    mz /= m_norm;

    T roll  = std::atan2(ay, az);
    T pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az));

    T s_roll = std::sin(roll), c_roll = std::cos(roll);
    T yaw = std::atan2(mz * s_roll - my * c_roll,
        mx * std::cos(pitch) + std::sin(pitch)*(my * s_roll + mz * c_roll));

    return quaternion::Quaternion<T>(utils::euler_to_quat(roll, pitch, yaw));
}
} // End namespace init

#endif
