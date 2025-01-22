/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <string>
#include <fstream>
#include <stdexcept>
#include <cmath>
#include <cstdlib>
#include <complex>
#include <limits>


#include <type_traits>
#include <memory>

#define ARILES2_IS_BASE_ENABLER(Base, Derived)                                                                         \
    const typename std::enable_if<std::is_base_of<Base, Derived>::value>::type * = nullptr

#include "build_config.h"

#include "exception.h"
#include "misc.h"


#define ARILES2_EMPTY_MACRO


#define ARILES2_BASIC_SIGNED_INTEGER_TYPES_LIST                                                                        \
    ARILES2_BASIC_TYPE(int)                                                                                            \
    ARILES2_BASIC_TYPE(short)                                                                                          \
    ARILES2_BASIC_TYPE(long)                                                                                           \
    ARILES2_BASIC_TYPE(long long)                                                                                      \
    ARILES2_BASIC_TYPE(char)

#define ARILES2_BASIC_UNSIGNED_INTEGER_TYPES_LIST_WITHOUT_BYTE                                                         \
    ARILES2_BASIC_TYPE(unsigned int)                                                                                   \
    ARILES2_BASIC_TYPE(unsigned short)                                                                                 \
    ARILES2_BASIC_TYPE(unsigned long)                                                                                  \
    ARILES2_BASIC_TYPE(unsigned long long)

#define ARILES2_BASIC_UNSIGNED_INTEGER_TYPES_LIST                                                                      \
    ARILES2_BASIC_UNSIGNED_INTEGER_TYPES_LIST_WITHOUT_BYTE                                                             \
    ARILES2_BASIC_TYPE(unsigned char)


#define ARILES2_BASIC_INTEGER_TYPES_LIST                                                                               \
    ARILES2_BASIC_SIGNED_INTEGER_TYPES_LIST                                                                            \
    ARILES2_BASIC_UNSIGNED_INTEGER_TYPES_LIST

#define ARILES2_BASIC_REAL_TYPES_LIST                                                                                  \
    ARILES2_BASIC_TYPE(float)                                                                                          \
    ARILES2_BASIC_TYPE(double)

#define ARILES2_BASIC_NUMERIC_TYPES_LIST                                                                               \
    ARILES2_BASIC_INTEGER_TYPES_LIST                                                                                   \
    ARILES2_BASIC_REAL_TYPES_LIST                                                                                      \
    ARILES2_BASIC_TYPE(bool)

#define ARILES2_COMPLEX_NUMBER_TYPES_LIST                                                                              \
    ARILES2_BASIC_TYPE(std::complex<float>)                                                                            \
    ARILES2_BASIC_TYPE(std::complex<double>)

#define ARILES2_BASIC_TYPES_LIST                                                                                       \
    ARILES2_BASIC_NUMERIC_TYPES_LIST                                                                                   \
    ARILES2_BASIC_TYPE(std::string)


#define CPPUT_COMPILE_SHARED_LIB  // we always build shared libs
#include "visibility.h"


// #define CPPUT_TRACE_ENABLE
#include "trace.h"


namespace ariles2
{
    // intentionally not defined
    template <class t_Pointer>
    class PointerHandler;


    template <class t_Entry>
    constexpr bool isMissing(const t_Entry & /*entry*/)
    {
        return (false);
    }


    class Ariles
    {
    protected:
        ~Ariles()
        {
        }
        Ariles()
        {
        }

    public:
        virtual const std::string &arilesDefaultID() const = 0;
    };
}  // namespace ariles2
