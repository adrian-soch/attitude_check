#pragma once

#include <Eigen/Dense>

#include "quaternion.hpp"

namespace attitude_check {
class AttitudeCheck {
public:
    AttitudeCheck(float imu_gain, float marg_gain);

    AttitudeCheck(float imu_gain, float marg_gain, float q0_w, float q0_x, float q0_y, float q0_z);

    quaternion::Quaternion<float> update(Eigen::Vector3f& acc, Eigen::Vector3f& gyr, Eigen::Vector3f& mag, float dt);

    quaternion::Quaternion<float> update(Eigen::Vector3f& acc, Eigen::Vector3f& gyr, float dt);

    /**
     * @brief Manually set the quaternion component. Useful when a new initial quaternion
     *  for subsequent calcualtion is desired. For example, after determining absolute
     *  orientation through another source, the quaternion can be reset such that the orientation
     *  is relative to the new known orientation.
     *
     * @param q_w
     * @param q_x
     * @param q_y
     * @param q_z
     */
    void set_quaternion(float q_w, float q_x, float q_y, float q_z);

    /**
     * @brief Set the filter gain values.
     *
     * @param imu_gain Gain when only Accel and Gyro are used.
     * @param marg_gain Gain when Accel, Gyro, and Mag are used.
     */
    void
    set_gain(float imu_gain, float marg_gain);

    /**
     * @brief Get the gains via tuple.
     *  Example:  `auto [imu_gain, marg_gain] = a.get_gain();`
     *
     * @return std::tuple<float, float>
     */
    std::tuple<float, float> get_gain();

private:
    const float RATE_MAX_HZ { 10000.0 };
    const float DT_MIN_SEC { 1 / RATE_MAX_HZ };
    const float GAIN_MIN { 0.0 }, GAIN_MAX { 1.0 };

    quaternion::Quaternion<float> m_q;

    float m_dt { 100.0 };
    float m_imu_gain { 0.5 }, m_marg_gain { 0.5 };
};
} // End namespace attitude_check
