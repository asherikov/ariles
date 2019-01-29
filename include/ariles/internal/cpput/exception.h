/**
    @file
    @author  Alexander Sherikov

    @copyright 2019 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Throw & assert macro
*/

#ifndef H_CPPUT_EXCEPTION
#define H_CPPUT_EXCEPTION

#include <string>

#define CPPUT_THROW_EXCEPTION(exception_type, message) throw exception_type((message))

#ifdef CMAKEUT_COMPILER_SUPPORTS_FUNC_
#    define CPPUT_THROW(s) CPPUT_THROW_EXCEPTION(std::runtime_error, (std::string("In ") + __func__ + "() // " + (s)))
#else  //
#    ifdef CMAKEUT_COMPILER_SUPPORTS_FUNCTION_
#        define CPPUT_THROW(s)                                                                                         \
            CPPUT_THROW_EXCEPTION(std::runtime_error, (std::string("In ") + __FUNCTION__ + "() // " + (s)))
#    else  //
#        define CPPUT_THROW(s) CPPUT_THROW_EXCEPTION(std::runtime_error, (s))
#    endif  //
#endif      //


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
