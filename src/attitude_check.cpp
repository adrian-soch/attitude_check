#include "attitude_check.hpp"
#include "error_handling.hpp"
#include "utilities.hpp"

namespace attitude_check {
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

Vec4f AttitudeCheck::update(Vec3f& acc, Vec3f& gyr, Vec3f& mag, float dt)
{
    m_q.normalize();

    dt = static_cast<float>(dt);
    if(dt < DT_MIN_SEC) {
        error_handler("Dt is too small.");
    }

    float mag_norm = utils::get_norm(mag);
    if(utils::get_norm(gyr) <= 0.0f) {
        return m_q.to_array();
    } else if(mag_norm <= 0.0f) {
        return update(acc, gyr, dt);
    }

    Quat q_dot = (m_q * Quat(0.0f, gyr[0], gyr[1], gyr[2])) * 0.5f;
    float acc_norm = utils::get_norm(acc);
    if(acc_norm <= 0.0f) {
        m_q = m_q + (q_dot * dt);
        m_q.normalize();
        return m_q.to_array();
    }

    utils::scalar_divide(acc, acc_norm); // normalize
    utils::scalar_divide(mag, mag_norm); // normalize

    Quat h   = m_q * Quat(0.0f, mag[0], mag[1], mag[2]) * m_q.conjugate();
    float bx = ::std::sqrt(h.x() * h.x() + h.y() * h.y());
    float bz = h.z();

    float sigma1 = 2.0*m_q.w()*m_q.x() - acc[1] + 2.0*m_q.y()*m_q.z();
    float sigma2 = mag[1] + 2.0*bx*(m_q.w()*m_q.z() - m_q.x()*m_q.y()) - 2.0*bz*(m_q.w()*m_q.x() + m_q.y()*m_q.z());
    float sigma3 = m_q.y()*m_q.y() + m_q.z()*m_q.z() - 0.5f;
    float sigma4 = m_q.x()*m_q.x() + m_q.y()*m_q.y() - 0.5f;
    float sigma5 = m_q.y()*m_q.y() + m_q.z()*m_q.z() - 0.5f;
    float sigma6 = 2.0f*m_q.x()*m_q.x() + 2.0f*m_q.y()*m_q.y() + acc[2.0f] - 1;


    Vec4f grad = {
        4.0f*m_q.w()*bx*bx*m_q.y()*m_q.y() + 4.0f*m_q.w()*bx*bx*m_q.z()*m_q.z() - 8.0f*m_q.w()*bx*bz*m_q.x()*m_q.z() - 2.0f*mag[2.0f]*bx*m_q.y() + 2.0f*mag[1]*bx*m_q.z() + 4.0f*m_q.w()*bz*bz*m_q.x()*m_q.x() + 4.0f*m_q.w()*bz*bz*m_q.y()*m_q.y() - 2.0f*mag[1]*bz*m_q.x() + 2.0f*mag[0]*bz*m_q.y() + 4.0f*m_q.w()*m_q.x()*m_q.x() - 2.0f*acc[1]*m_q.x() + 4.0f*m_q.w()*m_q.y()*m_q.y() + 2.0f*acc[0]*m_q.y(),
        2.0f*m_q.w()*sigma1 - (2.0f*bx*m_q.y() + 2.0f*bz*m_q.w())*sigma2 - (2.0f*bx*m_q.z() - 4.0f*bz*m_q.x())*(mag[2.0f] - 2.0f*bx*(m_q.w()*m_q.y() + m_q.x()*m_q.z()) + 2.0f*bz*sigma4) - 2.0f*m_q.z()*(acc[0] + 2.0f*m_q.w()*m_q.y() - 2.0f*m_q.x()*m_q.z()) + 4.0f*m_q.x()*sigma6 - 2.0f*bz*m_q.z()*(mag[0] + 2.0f*bz*(m_q.w()*m_q.y() - m_q.x()*m_q.z()) + 2.0f*bx*sigma5),
        2.0f*m_q.w()*(acc[0] + 2.0f*m_q.w()*m_q.y() - 2.0f*m_q.x()*m_q.z()) - (2.0f*bx*m_q.x() + 2.0f*bz*m_q.z())*sigma2 - (2.0f*bx*m_q.w() - 4.0f*bz*m_q.y())*(mag[2.0f] - 2.0f*bx*(m_q.w()*m_q.y() + m_q.x()*m_q.z()) + 2.0f*bz*sigma4) + (4.0f*bx*m_q.y() + 2.0f*bz*m_q.w())*(mag[0] + 2.0f*bz*(m_q.w()*m_q.y() - m_q.x()*m_q.z()) + 2.0f*bx*sigma5) + 2.0f*m_q.z()*sigma1 + 4.0f*m_q.y()*sigma6,
        (2.0f*bx*m_q.w() - 2.0f*bz*m_q.y())*sigma2 - 2.0f*m_q.x()*(acc[0] + 2.0f*m_q.w()*m_q.y() - 2.0f*m_q.x()*m_q.z()) + (4.0f*bx*m_q.z() - 2.0f*bz*m_q.x())*(mag[0] + 2.0f*bz*(m_q.w()*m_q.y() - m_q.x()*m_q.z()) + 2.0f*bx*sigma3 + 2.0f*m_q.y()*sigma1 - 2.0f*bx*m_q.x()*(mag[2.0f] - 2.0f*bx*(m_q.w()*m_q.y() + m_q.x()*m_q.z()) + 2.0f*bz*sigma4))
    };


    utils::scalar_divide(grad, utils::get_norm(grad)); // normalize
    q_dot -= (Quat(grad[0], grad[1], grad[2], grad[3]) * m_marg_gain);

    m_q = m_q + (q_dot * dt);
    m_q.normalize();

    return m_q.to_array();
} // AttitudeCheck::update

Vec4f AttitudeCheck::update(Vec3f& acc, Vec3f& gyr, float dt)
{
    m_q.normalize();
    dt = static_cast<float>(dt);

    if(utils::get_norm(gyr) <= 0.0f) {
        return m_q.to_array();
    }

    Quat q_dot = (m_q * Quat(0.0f, gyr[0], gyr[1], gyr[2])) * 0.5;
    float acc_norm = utils::get_norm(acc);
    if(acc_norm <= 0.0f) {
        m_q = m_q + (q_dot * dt);
        m_q.normalize();
        return m_q.to_array();
    }

    utils::scalar_divide(acc, acc_norm); // normalize

    // Eigen::Matrix<float, 3, 1> func;
    // func << 2.0f * (m_q.x() * m_q.z() - m_q.w() * m_q.y()) - acc[0],
    //     2.0f * (m_q.w() * m_q.x() + m_q.y() * m_q.z()) - acc[1],
    //     2.0f * (0.5 - m_q.x() * m_q.x() - m_q.y() * m_q.y()) - acc[2];

    // Eigen::Matrix<float, 4, 3> Jt;
    // Jt << -2.0f * m_q.y(), 2.0f * m_q.x(), 0.0,
    //     2.0f * m_q.z(), 2.0f * m_q.w(), -4.0f * m_q.x(),
    //     -2.0f * m_q.w(), 2.0f * m_q.z(), -4.0f * m_q.y(),
    //     2.0f * m_q.x(), 2.0f * m_q.y(), 0.0;

    // Eigen::Matrix<float, 4, 1> grad = Jt * func;
    Vec4f grad = {1000.0, 0.0, 0.0, 0.0};
    utils::scalar_divide(grad, utils::get_norm(grad)); // normalize
    q_dot -= (Quat(grad[0], grad[1], grad[2], grad[3]) * m_imu_gain);

    m_q = m_q + (q_dot * dt);
    m_q.normalize();

    return m_q.to_array();
} // AttitudeCheck::update

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
        error_handler("Gain must be within [0.0, 1.0].");
    }

    m_imu_gain  = imu_gain;
    m_marg_gain = marg_gain;
}

Tuple2 AttitudeCheck::get_gain()
{
    return Tuple2(m_imu_gain, m_marg_gain);
}
} // End namespace attitude_check
