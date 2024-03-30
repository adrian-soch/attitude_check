#pragma once

namespace attitude_check {
enum ErrorCodes {
    INVALID_ZERO_QUATERNION = 1,
};

#ifdef ARDUINO

# define LOG_OUTPUT SerialUSB

# include <EasyLogger.h>
inline void error_handler(const std::string& msg)
{
    LOG_ERROR("", msg.c_str());
}

#else

# include <stdexcept>
inline void error_handler(const std::string& msg)
{
    throw std::invalid_argument(msg);
}

#endif // ifdef ARDUINO
}
