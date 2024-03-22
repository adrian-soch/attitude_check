#include <stdexcept>

#include "attitude_check.hpp"

namespace q = quaternion;
typedef Eigen::Vector3f Vec3f;
typedef q::Quaternion<float> Quat;

namespace attitude_check {
AttitudeCheck::AttitudeCheck(float imu_gain, float marg_gain)
{
    if((imu_gain < 0.0f || imu_gain > 1.0f) ||
      (marg_gain < 0.0f || marg_gain > 1.0f))
    {
        throw std::invalid_argument("Gain must be within [0.0, 1.0].");
    }
}

AttitudeCheck::AttitudeCheck(float imu_gain, float marg_gain, float q0_w, float q0_x, float q0_y, float q0_z)
{
    if((imu_gain < 0.0f || imu_gain > 1.0f) ||
      (marg_gain < 0.0f || marg_gain > 1.0f))
    {
        throw std::invalid_argument("Gain must be within [0.0, 1.0].");
    }

    try {
        m_q.set(q0_w, q0_x, q0_y, q0_z);
    } catch (const std::invalid_argument& e) {
        throw std::invalid_argument("Initial quaternion must have a norm > 0.");
    }
}

Quat AttitudeCheck::update(Vec3f& acc, Vec3f& gyr, Vec3f& mag, float dt)
{
    dt = static_cast<float>(dt);

    if(gyr.norm() <= 0.0f) {
        return m_q;
    } else if(mag.norm() <= 0.0f) {
        return update(acc, gyr, dt);
    }

    Quat q_dot = (m_q * Quat(0.0f, gyr[0], gyr[1], gyr[2])) * 0.5f;
    if(acc.norm() <= 0.0f) {
        Quat q_out = m_q + (q_dot * dt);
        q_out.normalize();
        return q_out;
    }

    acc.normalize();
    mag.normalize();
    m_q.normalize();

    Quat h   = m_q * Quat(0.0f, mag[0], mag[1], mag[2]) * m_q.conjugate();
    float bx = std::sqrt(h.x() * h.x() + h.y() * h.y());
    float bz = h.z();

    Eigen::Matrix<float, 6, 1> func;
    func << 2.0f * (m_q.x() * m_q.z() - m_q.w() * m_q.y()) - acc[0],
        2.0f * (m_q.w() * m_q.x() - m_q.y() * m_q.z()) - acc[1],
        2.0f * (0.5f - m_q.x() * m_q.x() - m_q.y() * m_q.y()) - acc[2],
        2.0f * bx * (0.5f - m_q.y() * m_q.y() - m_q.z() * m_q.z()) + 2.0f * bz
        * (0.5f - m_q.x() * m_q.z() - m_q.w() * m_q.y()) - mag[0],
        2.0f * bx * (m_q.x() * m_q.x() - m_q.w() * m_q.z()) + 2.0f * bz
        * (m_q.w() * m_q.x() + m_q.y() * m_q.z()) - mag[1],
        2.0f * bx * (m_q.w() * m_q.y() + m_q.x() * m_q.z()) + 2.0f * bz
        * (0.5f - m_q.x() * m_q.x() - m_q.y() * m_q.y()) - mag[2];

    Eigen::Matrix<float, 4, 6> Jt;
    Jt << -2.0f * m_q.y(), 2.0f * m_q.x(), 0.0, -2.0f * bz * m_q.y(), -2.0f * bx * m_q.z() + 2.0f * bz * m_q.x(),
        2.0f * bx * m_q.y(),
        2.0f * m_q.z(), 2.0f * m_q.w(), -4.0f * m_q.x(), 2.0f * bz * m_q.z(), 2.0f * bx * m_q.y() + 2.0f * bz * m_q.w(),
        2.0f * bx * m_q.z() - 4.0f * bz * m_q.x(),
        2.0f * m_q.z(), 2.0f * m_q.w(), -4.0f * m_q.x(), 2.0f * bz * m_q.z(), 2.0f * bx * m_q.y() + 2.0f * bz * m_q.w(),
        2.0f * bx * m_q.z() - 4.0f * bz * m_q.x(),
        -2.0f * m_q.w(), 2.0f * m_q.z(), -4.0f * m_q.y(), -4.0f * bx * m_q.y() - 2.0f * bz * m_q.w(),
        2.0f * bx * m_q.x() + 2.0f * bz * m_q.z(),
        2.0f * bx * m_q.w() - 4.0f * bz * m_q.y(),
        2.0f * m_q.x(), 2.0f * m_q.y(), 0.0, -4.0f * bx * m_q.z() + 2.0f * bz * m_q.x(),
        -2.0f * bx * m_q.w() + 2.0f * bz * m_q.y(), 2.0f * bx * m_q.x();

    Eigen::Matrix<float, 4, 1> grad = Jt * func;
    q_dot -= (Quat(grad[0], grad[1], grad[2], grad[3]) * m_marg_gain);

    Quat q_next = m_q + q_dot * dt;
    q_next.normalize();

    return q_next;
} // AttitudeCheck::update

Quat AttitudeCheck::update(Vec3f& acc, Vec3f& gyr, float dt)
{
    dt = static_cast<float>(dt);

    if(gyr.norm() <= 0.0f) {
        return m_q;
    }

    Quat q_dot = (m_q * Quat(0.0f, gyr[0], gyr[1], gyr[2])) * 0.5;
    if(acc.norm() <= 0.0f) {
        Quat q_out = m_q + (q_dot * dt);
        q_out.normalize();
        return q_out;
    }

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
    q_dot -= (Quat(grad[0], grad[1], grad[2], grad[3]) * m_marg_gain);

    Quat q_next = m_q + q_dot * dt;
    q_next.normalize();

    return q_next;
} // AttitudeCheck::update

void AttitudeCheck::reset(float q_w, float q_x, float q_y, float q_z)
{
    m_q.set(q_w, q_x, q_y, q_z);
    m_q.normalize();
}
} // End namespace attitude_check
