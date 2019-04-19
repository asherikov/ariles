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
#include "cpput_config.h"
#include "cpput_exception.h"
#include "cpput_floating_point_utils.h"
#include "cpput_misc.h"
#include "cpput_flags.h"


#ifndef ARILES_DEFAULT_CONFIG_PREFIX
#   define ARILES_DEFAULT_CONFIG_PREFIX     ""
#endif

#define ARILES_INITIALIZE  "ariles/internal/define_accessors.h"



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
#   include "cpput_visibility.h"
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

#include "../configurable_flags.h"
#include "../bridge_flags.h"
