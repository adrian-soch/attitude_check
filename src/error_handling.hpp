// #ifndef ERROR_HANDLING_HPP

// #define ERROR_HANDLING_HPP

#pragma once

# define LOG_OUTPUT SerialUSB
using Msg = std::string;
namespace attitude_check {
#ifdef ARDUINO

# include <EasyLogger.h>
inline void error_handler(const Msg& msg)
{
    // LOG_ERROR("", msg.c_str());
}

#else

# include <stdexcept>
inline void error_handler(const Msg& msg)
{
    throw std::invalid_argument(msg);
}

#endif // ifdef ARDUINO
}

// #endif // ifndef ERROR_HANDLING_HPP
