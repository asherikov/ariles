/**
    @file
    @author  Alexander Sherikov

    @copyright 2019 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Throw & assert macro
*/

#ifndef H_CPPUT_EXCEPTION
#define H_CPPUT_EXCEPTION

#include <stdexcept>
#include "concat.h"

#define CPPUT_THROW_EXCEPTION(exception_type, message) throw exception_type((message))

#define CPPUT_THROW(...)                                                                                               \
    CPPUT_THROW_EXCEPTION(std::runtime_error, cpput::concat::simple("In ", __func__, "() // ", __VA_ARGS__))


#define CPPUT_PERSISTENT_ASSERT(condition, message)                                                                    \
    if (!(condition))                                                                                                  \
    {                                                                                                                  \
        CPPUT_THROW(message);                                                                                          \
    };

#ifdef DNDEBUG
#    define CPPUT_ASSERT(condition, message)
#else
#    define CPPUT_ASSERT(condition, message) CPPUT_PERSISTENT_ASSERT(condition, message)
#endif

#endif
