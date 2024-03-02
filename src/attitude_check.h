#pragma once

class AttitudeCheck
{
public:

    /**
     * @brief Construct a new Attitude Check object
     *
     * @param dt
     * @param gain
     * @param q0
     */
    AttitudeCheck(float dt, float gain, float q0[4]);

    ~AttitudeCheck();
private:

    float _dt { 100.0 };                   // [ms] time delta between measurements (sensor rate)
    float _gain { 0.5 };                   // filter gain parameter
    float _q0[4] = { 1.0, 0.0, 0.0, 0.0 }; // initial orientation in quaternion

    /**
     * @brief Check input values are valid
     *
     * @param dt
     * @param gain
     */
    void _input_handler(float dt, float gain);
};
