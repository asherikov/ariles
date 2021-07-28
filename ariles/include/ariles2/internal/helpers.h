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


#if __cplusplus >= 201103L

#    include <type_traits>
#    include <memory>

#    define ARILES2_IS_ENUM_ENABLER(Enum) const typename std::enable_if<(std::is_enum<Enum>::value)>::type * = NULL

#    define ARILES2_IS_FLOATING_POINT_ENABLER_TYPE(Real) std::enable_if<(std::is_floating_point<Real>::value)>::type *

#    define ARILES2_IS_BASE_OF(Base, Derived) std::is_base_of<Base, Derived>::value

#    define ARILES2_IS_BASE_ENABLER(Base, Derived)                                                                     \
        const typename std::enable_if<(ARILES2_IS_BASE_OF(Base, Derived))>::type * = NULL

#    define ARILES2_IS_BASE_DISABLER(Base, Derived)                                                                    \
        const typename std::enable_if<not(ARILES2_IS_BASE_OF(Base, Derived))>::type * = NULL

#    define ARILES2_IS_ANY_OF(Type, Type1, Type2)                                                                      \
        const typename std::enable_if<                                                                                 \
                std::is_base_of<Type1, Type>::value or std::is_base_of<Type2, Type>::value>::type * = NULL

#    define ARILES2_SHARED_PTR std::shared_ptr

#else

#    include <boost/utility/enable_if.hpp>
#    include <boost/type_traits/is_enum.hpp>
#    include <boost/type_traits/is_base_of.hpp>
#    include <boost/type_traits/is_floating_point.hpp>
#    include <boost/smart_ptr/shared_ptr.hpp>

#    define ARILES2_IS_ENUM_ENABLER(Enum)                                                                              \
        const typename boost::enable_if_c<(boost::is_enum<Enum>::value)>::type * = NULL

#    define ARILES2_IS_FLOATING_POINT_ENABLER_TYPE(Real)                                                               \
        boost::enable_if_c<(boost::is_floating_point<Real>::value)>::type *

#    define ARILES2_IS_BASE_OF(Base, Derived) boost::is_base_of<Base, Derived>::value

#    define ARILES2_IS_BASE_ENABLER(Base, Derived)                                                                     \
        const typename boost::enable_if_c<(ARILES2_IS_BASE_OF(Base, Derived))>::type * = NULL

#    define ARILES2_IS_BASE_DISABLER(Base, Derived)                                                                    \
        const typename boost::enable_if_c<not(ARILES2_IS_BASE_OF(Base, Derived))>::type * = NULL

#    define ARILES2_IS_ANY_OF(Type, Type1, Type2)                                                                      \
        const typename boost::enable_if_c<                                                                             \
                boost::is_base_of<Type1, Type>::value or boost::is_base_of<Type2, Type>::value>::type * = NULL

#    define ARILES2_SHARED_PTR boost::shared_ptr

#endif


#include "build_config.h"

#define CMAKEUT_COMPILER_SUPPORTS_FUNC_
#include "cpput_config.h"
#include "cpput_exception.h"
#include "cpput_misc.h"


#define ARILES2_EMPTY_MACRO


#if __cplusplus >= 201103L
#    define ARILES2_BASIC_SIGNED_INTEGER_TYPES_LIST                                                                    \
        ARILES2_BASIC_TYPE(int)                                                                                        \
        ARILES2_BASIC_TYPE(short)                                                                                      \
        ARILES2_BASIC_TYPE(long)                                                                                       \
        ARILES2_BASIC_TYPE(long long)                                                                                  \
        ARILES2_BASIC_TYPE(char)

#    define ARILES2_BASIC_UNSIGNED_INTEGER_TYPES_LIST                                                                  \
        ARILES2_BASIC_TYPE(unsigned int)                                                                               \
        ARILES2_BASIC_TYPE(unsigned short)                                                                             \
        ARILES2_BASIC_TYPE(unsigned long)                                                                              \
        ARILES2_BASIC_TYPE(unsigned long long)                                                                         \
        ARILES2_BASIC_TYPE(unsigned char)
#else
#    define ARILES2_BASIC_SIGNED_INTEGER_TYPES_LIST                                                                    \
        ARILES2_BASIC_TYPE(int)                                                                                        \
        ARILES2_BASIC_TYPE(short)                                                                                      \
        ARILES2_BASIC_TYPE(long)                                                                                       \
        ARILES2_BASIC_TYPE(char)

#    define ARILES2_BASIC_UNSIGNED_INTEGER_TYPES_LIST                                                                  \
        ARILES2_BASIC_TYPE(unsigned int)                                                                               \
        ARILES2_BASIC_TYPE(unsigned short)                                                                             \
        ARILES2_BASIC_TYPE(unsigned long)                                                                              \
        ARILES2_BASIC_TYPE(unsigned char)
#endif

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


#ifndef ARILES2_VISIBILITY_ATTRIBUTE
#    include "cpput_visibility.h"
#    define ARILES2_VISIBILITY_ATTRIBUTE ARILES2_LIB_EXPORT
#endif


// #define ARILES2_TRACE_ENABLE
#include "cpput_trace.h"


namespace ariles2
{
    // intentionally not defined
    template <class t_Pointer>
    class ARILES2_VISIBILITY_ATTRIBUTE PointerHandler;


    class ARILES2_VISIBILITY_ATTRIBUTE Ariles
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
