/**
 * @file attitude_check.hpp
 * @brief Interface for the Attitude Check Orientation Estimator
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
// #ifndef ATTITUDE_CHECK_HPP
// #define ATTITUDE_CHECK_HPP

#include <array>
#include <tuple>

#include "quaternion.hpp"

using Vec3f = std::array<float, 3>;
using Vec4f = std::array<float, 4>;
using Quat = quaternion::Quaternion<float>;
using Tuple2 = std::tuple<float, float>;

namespace attitude_check {
class AttitudeCheck {
public:

    /**
     * @brief Construct a new Attitude Check object with default filter gain.
     *  Initial position will be calculated from the first set of sensor measurements.
     *
     */
    AttitudeCheck();

    /**
     * @brief Construct a new Attitude Check object with custom gain values.
     *  Initial position will be calculated from the first set of sensor measurements.
     *
     * @param imu_gain
     * @param marg_gain
     */
    AttitudeCheck(float imu_gain, float marg_gain);

    /**
     * @brief Construct a new Attitude Check object with custom gain, and initial
     *  orientation estimate.
     *
     * @param imu_gain
     * @param marg_gain
     * @param q0_w float
     * @param q0_x float
     * @param q0_y float
     * @param q0_z float
     */
    AttitudeCheck(float imu_gain, float marg_gain, float q0_w, float q0_x, float q0_y, float q0_z);

    /**
     * @brief Estimate the next sequencial orientation using Accel, Mag, and Gyro measurements.
     *
     * @param acc Vec3f m/s^2
     * @param gyr Vec3f rad/s
     * @param mag Vec3f uT (micro-Tesla)
     * @param dt Time since last call to `update()` in seconds.
     * @return quaternion::Quaternion<float>
     */
    Vec4f
    update(Vec3f& acc, Vec3f& gyr, Vec3f& mag, float dt);

    /**
     * @brief Estimate the next sequencial orientation using Accel, and Gyro measurements.
     *
     * @param acc Vec3f m/s^2
     * @param gyr Vec3f rad/s
     * @param dt Time since last call to `update()` in seconds.
     * @return quaternion::Quaternion<float>
     */
    Vec4f
    update(Vec3f& acc, Vec3f& gyr, float dt);

    /**
     * @brief Manually set the quaternion component. Useful when a new initial quaternion
     *  for subsequent calcualtion is desired. For example, after determining absolute
     *  orientation through another source, the quaternion can be reset such that the orientation
     *  is relative to the new known orientation.
     *
     * @param q_w float
     * @param q_x float
     * @param q_y float
     * @param q_z float
     */
    void
    set_quaternion(float q_w, float q_x, float q_y, float q_z);

    /**
     * @brief Set the filter gain values.
     *
     * @param imu_gain Gain when only Accel and Gyro are used [0.0, 1.0].
     * @param marg_gain Gain when Accel, Gyro, and Mag are used [0.0, 1.0].
     */
    void
    set_gain(float imu_gain, float marg_gain);

    /**
     * @brief Get the gains via tuple.
     *  Example:  `auto [imu_gain, marg_gain] = a.get_gain();`
     *
     * @return std::tuple<float, float>
     */
    Tuple2
    get_gain();

private:
    const float RATE_MAX_HZ { 10000.0 };
    const float DT_MIN_SEC { 1 / RATE_MAX_HZ };
    const float GAIN_MIN { 0.0 }, GAIN_MAX { 1.0 };

    Quat m_q;

    float m_dt { 100.0 };
    float m_imu_gain { 0.033 }, m_marg_gain { 0.041 };
};
} // End namespace attitude_check

// #endif // ifndef ATTITUDE_CHECK_HPP
