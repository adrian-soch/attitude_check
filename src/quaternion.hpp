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

#include "error_handling.hpp"

namespace quaternion {
enum Axis {
    W = 0,
    X,
    Y,
    Z
};

template<typename T>
class Quaternion {
public:
    Quaternion() : m_q{static_cast<T>(1.0), static_cast<T>(0.0),
          static_cast<T>(0.0), static_cast<T>(0.0)}{ };

    Quaternion(T w, T x, T y, T z) : m_q{w, x, y, z}
    {
        if((std::abs(w) + std::abs(x) + std::abs(y) + std::abs(z)) <= EPS) {
            attitude_check::error_handler("Cannot create quaternion: Magnitude of quaternion cannot be zero.");
        }
    }

    Quaternion(const Quaternion& b)
    {
        m_q[W] = b.w();
        m_q[X] = b.x();
        m_q[Y] = b.y();
        m_q[Z] = b.z();
    }

    /**
     * @brief Overload the assigment operator for this class
     *
     * @param other
     * @return Quaternion&
     */
    Quaternion& operator = (const Quaternion& b)
    {
        if(this != &b) {
            m_q[W] = b.w();
            m_q[X] = b.x();
            m_q[Y] = b.y();
            m_q[Z] = b.z();
        }
        return *this;
    }

    /**
     * @brief Set quaternion values
     *
     * @param w float/double
     * @param x float/double
     * @param y float/double
     * @param z float/double
     */
    inline void set(const T w, const T x, const T y, const T z)
    {
        if((std::abs(w) + std::abs(x) + std::abs(y) + std::abs(z)) <= EPS) {
            attitude_check::error_handler("Cannot set quaternion: Magnitude of quaternion cannot be zero.");
        }
        m_q[W] = w;
        m_q[X] = x;
        m_q[Y] = y;
        m_q[Z] = z;
    }

    /**
     * @brief Getter for w component.
     *
     * @return const T&: w
     */
    inline const T& w() const { return m_q[W]; }

    /**
     * @brief Getter for the x component.
     *
     * @return const T& : x
     */
    inline const T& x() const { return m_q[X]; }

    /**
     * @brief Getter for the y component.
     *
     * @return const T& : y
     */
    inline const T& y() const { return m_q[Y]; }

    /**
     * @brief Getter for the z component.
     *
     * @return const T& : z
     */
    inline const T& z() const { return m_q[Z]; }

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
        return Quaternion( (m_q[W] * b.w() - m_q[X] * b.x() - m_q[Y] * b.y() - m_q[Z] * b.z()),
                 (m_q[W] * b.x() + m_q[X] * b.w() + m_q[Y] * b.z() - m_q[Z] * b.y()),
                 (m_q[W] * b.y() + m_q[Y] * b.w() + m_q[Z] * b.x() - m_q[X] * b.z()),
                 (m_q[W] * b.z() + m_q[Z] * b.w() + m_q[X] * b.y() - m_q[Y] * b.x()));
    }

    /**
     * @brief Overload the * (multiplication operator) to perform
     *  scalar multiplication (scalar must be on the right)
     *
     * @param k float/double
     * @return Quaternion
     */
    inline Quaternion operator * (T const& k) const
    {
        return Quaternion(k * m_q[W], k * m_q[X], k * m_q[Y], k * m_q[Z]);
    }

    /**
     * @brief Overload the + operator to add 2 Quaternions.
     *
     * @param b Quaternion
     * @return Quaternion
     */
    inline Quaternion operator + (Quaternion const& b) const
    {
        return Quaternion(m_q[W] + b.w(), m_q[X] + b.x(), m_q[Y] + b.y(), m_q[Z] + b.z());
    }

    /**
     * @brief Overload the - operator to subtract 2 Quaternions.
     *
     * @param b Quaternion
     * @return Quaternion
     */
    inline Quaternion operator - (Quaternion const& b) const
    {
        return Quaternion(m_q[W] - b.w(), m_q[X] - b.x(), m_q[Y] - b.y(), m_q[Z] - b.z());
    }

    /**
     * @brief Overload the -= operator to subtract 2 Quaternions.
     *
     * @param b Quaternion
     * @return Quaternion
     */
    inline void operator -= (Quaternion const& b)
    {
        m_q[W] -= b.w();
        m_q[X] -= b.x();
        m_q[Y] -= b.y();
        m_q[Z] -= b.z();
    }

    /**
     * @brief Normalize the quaternion such that it becomes a unit quaternion. L2 norm = 1.
     *
     */
    inline void normalize()
    {
        T euclid_dist = std::sqrt(m_q[W] * m_q[W] + m_q[X] * m_q[X] + m_q[Y] * m_q[Y] + m_q[Z] * m_q[Z]);

        m_q[W] /= euclid_dist;
        m_q[X] /= euclid_dist;
        m_q[Y] /= euclid_dist;
        m_q[Z] /= euclid_dist;
    }

    /**
     * @brief Return quaternion as an array
     *
     * @return std::array<float, 4>
     */
    inline std::array<float, 4> to_array() const
    {
        return std::array<float, 4> { m_q[W], m_q[X], m_q[Y], m_q[Z] };
    }

private:
    const T EPS { 0.0000001 };
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

// template <>
// void Quaternion<float>::normalize() {
//     float i_sqrr = fast_inv_sqrt(std::pow(m_q[0], 2.0) + std::pow(m_q[1], 2.0)
//             + std::pow(m_q[Y], 2.0) + std::pow(m_q[Z], 2.0));

//     m_q[W] *= i_sqrr;
//     m_q[X] *= i_sqrr;
//     m_q[Y] *= i_sqrr;
//     m_q[Z] *= i_sqrr;
// }
} // End namespace quaternion
