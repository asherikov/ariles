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
#include <boost/type_traits/is_base_of.hpp>

#include "build_config.h"

#ifndef ARILES_DEFAULT_CONFIG_PREFIX
#   define ARILES_DEFAULT_CONFIG_PREFIX     ""
#endif

#define ARILES_INITIALIZE  "ariles/internal/define_accessors.h"

#define ARILES_IGNORE_UNUSED(parameter)   (void)parameter;

#define ARILES_THROW_MSG(s) \
    throw std::runtime_error(std::string("In ") + __FILE__ + " // " + (s))

#ifdef DNDEBUG
#   define ARILES_ASSERT(condition, message)
#else
#   define ARILES_ASSERT(condition, message) if (!(condition)) {ARILES_THROW_MSG(message);};
#endif


#define ARILES_MACRO_SUBSTITUTE(arg) arg


#define ARILES_IS_ENUM_ENABLER_TYPE(Enum) \
    const typename boost::enable_if_c< (boost::is_enum<Enum>::value) >::type

#define ARILES_IS_CHILD_ENABLER_TYPE(parent, child) \
    const typename boost::enable_if_c< (boost::is_base_of<parent, child>::value) >::type


#define ARILES_BASIC_TYPES_LIST \
    ARILES_BASIC_TYPE(std::string) \
    ARILES_BASIC_TYPE(float) \
    ARILES_BASIC_TYPE(double) \
    ARILES_BASIC_TYPE(int) \
    ARILES_BASIC_TYPE(unsigned int) \
    ARILES_BASIC_TYPE(short) \
    ARILES_BASIC_TYPE(unsigned short) \
    ARILES_BASIC_TYPE(long) \
    ARILES_BASIC_TYPE(unsigned long) \
    ARILES_BASIC_TYPE(char) \
    ARILES_BASIC_TYPE(unsigned char) \
    ARILES_BASIC_TYPE(bool)


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
    };


    class ConfigurableParameters
    {
        public:
            bool crash_on_missing_entry_;
            bool enable_sloppy_maps_if_supported_;
            bool compact_arrays_if_supported_;

        public:
            ConfigurableParameters(const bool crash_on_missing_entry)
            {
                crash_on_missing_entry_ = crash_on_missing_entry;
#ifdef ARILES_ENABLE_SLOPPY_MAP
                enable_sloppy_maps_if_supported_ = true;
#else
                enable_sloppy_maps_if_supported_ = false;
#endif
                compact_arrays_if_supported_ = false;
            }
    };
}
