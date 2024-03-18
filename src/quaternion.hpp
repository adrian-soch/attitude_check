/**
 * @file quaternion.hpp
 * @brief
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
        if(std::sqrt(w * w + x * x + y * y + z * z) == 0.0f) {
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

    Quaternion operator * (Quaternion const& b) const
    {
        return Quaternion ( (m_q[0] * b.w() - m_q[1] * b.x() - m_q[2] * b.y() - m_q[3] * b.z()),
                 (m_q[0] * b.x() + m_q[1] * b.w() + m_q[2] * b.z() - m_q[3] * b.y()),
                 (m_q[0] * b.y() + m_q[2] * b.w() + m_q[3] * b.x() - m_q[1] * b.z()),
                 (m_q[0] * b.z() + m_q[3] * b.w() + m_q[1] * b.y() - m_q[2] * b.x()));
    }

private:
    std::array<T, 4> m_q;
};

// /**
//  * @brief Compute Hamiltonian product between two Quaternions
//  *
//  * @param a
//  * @param b
//  * @return Quaternion c = hamilton_product(a, b)
//  */
// Quaternion quat_prod(const Quaternion& a, const Quaternion& b)
// {
//     return Quaternion prod(a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z()
//              a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y(),
//              a.w() * b.y() + a.y() * b.w() + a.z() * b.x() - a.x() * b.z(),
//              a.w() * b.z() + a.z() * b.w() + a.x() * b.y() - a.y() * b.x());
// }

} // End namespace quaternion
