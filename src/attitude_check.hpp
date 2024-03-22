#pragma once

#include <Eigen/Dense>

#include "quaternion.hpp"

namespace attitude_check {
class AttitudeCheck {
public:
    AttitudeCheck(float imu_gain, float marg_gain);

    AttitudeCheck(float imu_gain, float marg_gain, float q0_w, float q0_x, float q0_y, float q0_z);

    quaternion::Quaternion<float> update(Eigen::Vector3f acc, Eigen::Vector3f gyr, Eigen::Vector3f mag, float dt);

    quaternion::Quaternion<float> update(Eigen::Vector3f acc, Eigen::Vector3f gyr, float dt);

    void reset(float q_w, float q_x, float q_y, float q_z);

private:
    const float RATE_MIN_HZ { 10000.0 };
    const float DT_MIN_SEC { 1 / RATE_MIN_HZ };
    const float GAIN_MIN { 0.0 }, GAIN_MAX { 1.0 };

    quaternion::Quaternion<float> m_q;

    float m_dt { 100.0 };
    float m_imu_gain { 0.5 }, m_marg_gain { 0.5 };

    void
    input_handler(float dt, float gain);

    void
    set_gain(float imu_gain, float marg_gain);
};
} // End namespace attitude_check
