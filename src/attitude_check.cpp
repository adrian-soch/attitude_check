#include <cassert>

#include "attitude_check.h"

AttitudeCheck::AttitudeCheck(float dt, float gain, float q0[4])
{
    _input_handler(dt, gain);
    _q0[0] = q0[0];
    _q0[1] = q0[1];
    _q0[2] = q0[2];
    _q0[3] = q0[3];
}

AttitudeCheck::~AttitudeCheck(){ }

void AttitudeCheck::_input_handler(float dt, float gain)
{
    assert(dt > 0.0); // check time is positive
    assert((gain >= 0.0) && (gain <= 1.0)); // check gain is within bounds

    _dt = dt;
    _gain = gain;
}
