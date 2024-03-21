/**
 * @file quaternion.hpp
 * @brief Quaternion Module provides the quaternion functionality and common operations.
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

namespace quaternion {
template<typename T>
class Quaternion {
public:
    Quaternion(T w, T x, T y, T z) : m_q{w, x, y, z}
    {
        if((std::abs(w) + std::abs(x) + std::abs(y) + std::abs(z)) <= EPS) {
            throw std::invalid_argument("Magnitude of quaternion cannot be zero.");
        }
    }

    /**
     * @brief Getter for w component.
     *
     * @return const T&: w
     */
    inline const T& w() const { return m_q[0]; }

    /**
     * @brief Getter for the x component.
     *
     * @return const T& : x
     */
    inline const T& x() const { return m_q[1]; }

    /**
     * @brief Getter for the y component.
     *
     * @return const T& : y
     */
    inline const T& y() const { return m_q[2]; }

    /**
     * @brief Getter for the z component.
     *
     * @return const T& : z
     */
    inline const T& z() const { return m_q[3]; }

    /**
     * @brief Compute the conjugate of the Quaternion object.
     *
     * @return Quaternion
     */
    inline Quaternion conjugate() const
    {
        return Quaternion(w(), -x(), -y(), -z());
    }

    /**
     * @brief Overload the * (multiplication operator) to perform
     *  quaternion multiplication (hamiltonian product)
     *
     * @param b Quaternion
     * @return Quaternion
     */
    inline Quaternion operator * (Quaternion const& b) const
    {
        return Quaternion( (m_q[0] * b.w() - m_q[1] * b.x() - m_q[2] * b.y() - m_q[3] * b.z()),
                 (m_q[0] * b.x() + m_q[1] * b.w() + m_q[2] * b.z() - m_q[3] * b.y()),
                 (m_q[0] * b.y() + m_q[2] * b.w() + m_q[3] * b.x() - m_q[1] * b.z()),
                 (m_q[0] * b.z() + m_q[3] * b.w() + m_q[1] * b.y() - m_q[2] * b.x()));
    }

    /**
     * @brief Normalize the quaternion such that it becomes a unit quaternion. L2 norm = 1.
     *
     */
    inline void normalize()
    {
        T euclid_dist = std::sqrt(
            std::pow(m_q[0], 2.0) + std::pow(m_q[1], 2.0)
            + std::pow(m_q[2], 2.0) + std::pow(m_q[3], 2.0));

        m_q[0] /= euclid_dist;
        m_q[1] /= euclid_dist;
        m_q[2] /= euclid_dist;
        m_q[3] /= euclid_dist;
    }

private:
    const T EPS {0.0000001};
    std::array<T, 4> m_q;
};

// float fast_inv_sqrt(float x) {
//     float halfx = 0.5f * x;
//     float y = x;
//     long i = *(long*)&y;
//     i = 0x5f3759df - (i>>1);
//     y = *(float*)&i;
//     y = y * (1.5f - (halfx * y * y));
//     y = y * (1.5f - (halfx * y * y));
//     return y;
// }

// float fast_inv_sqrt(float x) {
//     union {
//         int i;
//         float y;
//         long z;
//     } data;

//     // float data;

//     float halfx = 0.5f * x;
//     data.y = x;
//     long i = *(long*)&data.y;
//     i = 0x5f3759df - (i>>1);
//     data.y = *(float*)&i;
//     data.y = data.y * (1.5f - (halfx * data.y * data.y));
//     data.y = data.y * (1.5f - (halfx * data.y * data.y));
//     return data.y;
// }

// template <>
// void Quaternion<float>::normalize() {
//     float i_sqrr = fast_inv_sqrt(std::pow(m_q[0], 2.0) + std::pow(m_q[1], 2.0)
//             + std::pow(m_q[2], 2.0) + std::pow(m_q[3], 2.0));

//     m_q[0] *= i_sqrr;
//     m_q[1] *= i_sqrr;
//     m_q[2] *= i_sqrr;
//     m_q[3] *= i_sqrr;
// }
} // End namespace quaternion
