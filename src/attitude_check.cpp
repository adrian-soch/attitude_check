#include "attitude_check.hpp"
#include "error_handling.hpp"
#include "initializers.hpp"

namespace attitude_check {
typedef Eigen::Vector3f Vec3f;
typedef quaternion::Quaternion<float> Quat;

AttitudeCheck::AttitudeCheck(){ }

AttitudeCheck::AttitudeCheck(float imu_gain, float marg_gain)
{
    set_gain(imu_gain, marg_gain);
}

AttitudeCheck::AttitudeCheck(float imu_gain, float marg_gain, float q0_w, float q0_x, float q0_y, float q0_z)
{
    set_gain(imu_gain, marg_gain);
    set_quaternion(q0_w, q0_x, q0_y, q0_z);
}

std::array<float, 4> AttitudeCheck::update(Vec3f& acc, Vec3f& gyr, Vec3f& mag, float dt)
{
    m_q.normalize();

    dt = static_cast<float>(dt);
    if(dt < DT_MIN_SEC) {
        attitude_check::error_handler("Dt is too small.");
    }

    if(gyr.norm() <= 0.0f) {
        return m_q.to_array();
    } else if(mag.norm() <= 0.0f) {
        return update(acc, gyr, dt);
    }

    Quat q_dot = (m_q * Quat(0.0f, gyr[0], gyr[1], gyr[2])) * 0.5f;
    if(acc.norm() <= 0.0f) {
        m_q = m_q + (q_dot * dt);
        m_q.normalize();
        return m_q.to_array();
    }

    acc.normalize();
    mag.normalize();

    Quat h   = m_q * Quat(0.0f, mag[0], mag[1], mag[2]) * m_q.conjugate();
    float bx = std::sqrt(h.x() * h.x() + h.y() * h.y());
    float bz = h.z();

    /* *INDENT-OFF* */
    Eigen::Matrix<float, 6, 1> func;
    func << 2.0f * (m_q.x() * m_q.z() - m_q.w() * m_q.y()) - acc[0],
        2.0f * (m_q.w() * m_q.x() + m_q.y() * m_q.z()) - acc[1],
        2.0f * (0.5f - m_q.x() * m_q.x() - m_q.y() * m_q.y()) - acc[2],
        2.0f * bx *(0.5f - m_q.y() * m_q.y() - m_q.z() * m_q.z()) + 2.0f * bz * (m_q.x() * m_q.z() - m_q.w() * m_q.y()) - mag[0],
        2.0f * bx * (m_q.x() * m_q.y() - m_q.w() * m_q.z()) + 2.0f * bz * (m_q.w() * m_q.x() + m_q.y() * m_q.z()) - mag[1],
        2.0f * bx * (m_q.w() * m_q.y() + m_q.x() * m_q.z()) + 2.0f * bz * (0.5f - m_q.x() * m_q.x() - m_q.y() * m_q.y()) - mag[2];

    Eigen::Matrix<float, 4, 6> Jt;
    Jt << -2.0f * m_q.y(), 2.0f * m_q.x(), 0.0f, -2.0f * bz * m_q.y(), -2.0f * bx * m_q.z() + 2.0f * bz * m_q.x(), 2.0f * bx * m_q.y(),

        2.0f * m_q.z(), 2.0f * m_q.w(), -4.0f * m_q.x(), 2.0f * bz * m_q.z(), 2.0f * bx * m_q.y() + 2.0f * bz * m_q.w(), 2.0f * bx * m_q.z() - 4.0f * bz * m_q.x(),

        -2.0f * m_q.w(), 2.0f * m_q.z(), -4.0f * m_q.y(), -4.0f * bx * m_q.y() - 2.0f * bz * m_q.w(), 2.0f * bx * m_q.x() + 2.0f * bz * m_q.z(), 2.0f * bx * m_q.w() - 4.0f * bz * m_q.y(),

        2.0f * m_q.x(), 2.0f * m_q.y(), 0.0f, -4.0f * bx * m_q.z() + 2.0f * bz * m_q.x(), -2.0f * bx * m_q.w() + 2.0f * bz * m_q.y(), 2.0f * bx * m_q.x();
    /* *INDENT-ON* */

    Eigen::Matrix<float, 4, 1> grad = Jt * func;
    grad.normalize();
    q_dot -= (Quat(grad[0], grad[1], grad[2], grad[3]) * m_marg_gain);

    m_q = m_q + (q_dot * dt);
    m_q.normalize();

    return m_q.to_array();
} // AttitudeCheck::update

std::array<float, 4> AttitudeCheck::update(Vec3f& acc, Vec3f& gyr, float dt)
{
    m_q.normalize();
    dt = static_cast<float>(dt);

    if(gyr.norm() <= 0.0f) {
        return m_q.to_array();
    }

    Quat q_dot = (m_q * Quat(0.0f, gyr[0], gyr[1], gyr[2])) * 0.5;
    if(acc.norm() <= 0.0f) {
        m_q = m_q + (q_dot * dt);
        m_q.normalize();
        return m_q.to_array();
    }

    acc.normalize();

    Eigen::Matrix<float, 3, 1> func;
    func << 2.0f * (m_q.x() * m_q.z() - m_q.w() * m_q.y()) - acc[0],
        2.0f * (m_q.w() * m_q.x() + m_q.y() * m_q.z()) - acc[1],
        2.0f * (0.5 - m_q.x() * m_q.x() - m_q.y() * m_q.y()) - acc[2];

    Eigen::Matrix<float, 4, 3> Jt;
    Jt << -2.0f * m_q.y(), 2.0f * m_q.x(), 0.0,
        2.0f * m_q.z(), 2.0f * m_q.w(), -4.0f * m_q.x(),
        -2.0f * m_q.w(), 2.0f * m_q.z(), -4.0f * m_q.y(),
        2.0f * m_q.x(), 2.0f * m_q.y(), 0.0;

    Eigen::Matrix<float, 4, 1> grad = Jt * func;
    grad.normalize();
    q_dot -= (Quat(grad[0], grad[1], grad[2], grad[3]) * m_imu_gain);

    m_q = m_q + (q_dot * dt);
    m_q.normalize();

    return m_q.to_array();
} // AttitudeCheck::update

void AttitudeCheck::get_initial_orientation(Vec3f& acc, Vec3f& mag)
{
    auto q0 = initializers::mag_to_quat(acc, mag);

    set_quaternion(q0.w(), q0.x(), q0.y(), q0.z());
}

void AttitudeCheck::get_initial_orientation(Vec3f& acc)
{
    auto q0 = initializers::acc_to_quat(acc);

    set_quaternion(q0.w(), q0.x(), q0.y(), q0.z());
}

void AttitudeCheck::set_quaternion(float q_w, float q_x, float q_y, float q_z)
{
    m_q.set(q_w, q_x, q_y, q_z);
    m_q.normalize();
}

void AttitudeCheck::set_gain(const float imu_gain, const float marg_gain)
{
    if((imu_gain < GAIN_MIN || imu_gain > GAIN_MAX) ||
      (marg_gain < GAIN_MIN || marg_gain > GAIN_MAX))
    {
        attitude_check::error_handler("Gain must be within [0.0, 1.0].");
    }

    m_imu_gain  = imu_gain;
    m_marg_gain = marg_gain;
}

std::tuple<float, float> AttitudeCheck::get_gain()
{
    return std::tuple<float, float>(m_imu_gain, m_marg_gain);
}
} // End namespace attitude_check
