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


#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_enum.hpp>


#include "build_config.h"
#include "floating_point_utils.h"


#ifndef ARILES_DEFAULT_CONFIG_PREFIX
#   define ARILES_DEFAULT_CONFIG_PREFIX     ""
#endif

#define ARILES_INITIALIZE  "ariles/internal/define_accessors.h"

#define ARILES_IGNORE_UNUSED(parameter)   (void)parameter;


#ifdef ARILES_COMPILER_SUPPORTS_FUNC_
    #define ARILES_THROW_MSG(s) throw std::runtime_error(std::string("In ") + __func__ + "() // " + (s))
#else
    #ifdef ARILES_COMPILER_SUPPORTS_FUNCTION_
        #define ARILES_THROW_MSG(s) throw std::runtime_error(std::string("In ") + __FUNCTION__ + "() // " + (s))
    #else
        #define ARILES_THROW_MSG(s) throw std::runtime_error(s)
    #endif
#endif // ARILES_COMPILER_SUPPORTS_FUNC_



#ifdef DNDEBUG
#   define ARILES_ASSERT(condition, message)
#else
#   define ARILES_ASSERT(condition, message) if (!(condition)) {ARILES_THROW_MSG(message);};
#endif


#define ARILES_MACRO_SUBSTITUTE(arg) arg


#define ARILES_IS_ENUM_ENABLER_TYPE(Enum) \
    const typename boost::enable_if_c< (boost::is_enum<Enum>::value) >::type


#define ARILES_BASIC_SIGNED_INTEGER_TYPES_LIST \
    ARILES_BASIC_TYPE(int) \
    ARILES_BASIC_TYPE(short) \
    ARILES_BASIC_TYPE(long) \
    ARILES_BASIC_TYPE(char)

#define ARILES_BASIC_UNSIGNED_INTEGER_TYPES_LIST \
    ARILES_BASIC_TYPE(unsigned int) \
    ARILES_BASIC_TYPE(unsigned short) \
    ARILES_BASIC_TYPE(unsigned long) \
    ARILES_BASIC_TYPE(unsigned char)

#define ARILES_BASIC_INTEGER_TYPES_LIST \
    ARILES_BASIC_SIGNED_INTEGER_TYPES_LIST \
    ARILES_BASIC_UNSIGNED_INTEGER_TYPES_LIST

#define ARILES_BASIC_REAL_TYPES_LIST \
    ARILES_BASIC_TYPE(float) \
    ARILES_BASIC_TYPE(double) \

#define ARILES_BASIC_NUMERIC_TYPES_LIST \
    ARILES_BASIC_INTEGER_TYPES_LIST \
    ARILES_BASIC_REAL_TYPES_LIST \
    ARILES_BASIC_TYPE(bool)

#define ARILES_BASIC_TYPES_LIST \
    ARILES_BASIC_NUMERIC_TYPES_LIST \
    ARILES_BASIC_TYPE(std::string)


#ifndef ARILES_VISIBILITY_ATTRIBUTE
// helper macro depending on the compiler
#   if defined _WIN32 || defined __CYGWIN__
#       define ARILES_LIB_IMPORT __declspec(dllimport)
#       define ARILES_LIB_EXPORT __declspec(dllexport)
#       define ARILES_LIB_LOCAL
#   else
#       if __GNUC__ >= 4
#           define ARILES_LIB_IMPORT __attribute__ ((visibility ("default")))
#           define ARILES_LIB_EXPORT __attribute__ ((visibility ("default")))
#           define ARILES_LIB_LOCAL  __attribute__ ((visibility ("hidden")))
#       else
#           define ARILES_LIB_IMPORT
#           define ARILES_LIB_EXPORT
#           define ARILES_LIB_LOCAL
#       endif
#   endif

#   define ARILES_VISIBILITY_ATTRIBUTE ARILES_LIB_EXPORT
#endif


namespace ariles
{
    struct ARILES_VISIBILITY_ATTRIBUTE BridgeSelectorBase
    {
        public:
            typedef int BridgeSelectorIndicatorType;
    };
}



namespace ariles
{
    template <class t_Derived>
    class Flags
    {
        public:
            enum Action
            {
                DEFAULT = 0,
                REPLACE = 1,
                SET = 2,
                UNSET = 3
            };


        public:
            int flags_;


        public:
            void initialize(const int flags, const Action action_type = REPLACE)
            {
                switch(action_type)
                {
                    case REPLACE:
                        replace(flags);
                        break;

                    case SET:
                        static_cast<t_Derived *>(this)->setDefaults();
                        set(flags);
                        break;

                    case UNSET:
                        static_cast<t_Derived *>(this)->setDefaults();
                        unset(flags);
                        break;

                    default:
                        ARILES_THROW_MSG("Unknown Flags::Action type.");
                }
            }


            void copy(const t_Derived & from, const int flags)
            {
                set(from.flags_ & flags);
            }

            bool isSet(const int flags) const
            {
                return (flags_ & flags);
            }

            void replace(const int flags)
            {
                flags_ = flags;
            }

            void set(const int flags)
            {
                flags_ |= flags;
            }

            void unset(const int flags)
            {
                flags_ &= !flags;
            }
    };
}

#include "../configurable_flags.h"
#include "../bridge_flags.h"
