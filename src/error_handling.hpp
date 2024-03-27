#pragma once

namespace attitude_check {
enum ErrorCodes {
    INVALID_ZERO_QUATERNION = 1,
};

#ifdef ARDUINO

# include <DebugLog.h>
inline void error_handler(const std::string& msg)
{
    LOG_ERROR(msg);
}

#else

# include <stdexcept>
inline void error_handler(const std::string& msg)
{
    throw std::invalid_argument(msg);
}

#endif // ifdef ARDUINO
}
