/**
    @file
    @author  Alexander Sherikov

    @copyright 2019 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Visibility defines, e.g., https://gcc.gnu.org/wiki/Visibility
*/

#pragma once

#ifndef H_CPPUT_VISIBILITY
#    define H_CPPUT_VISIBILITY

#    if defined _WIN32 || defined __CYGWIN__
#        define CPPUT_LIB_IMPORT __declspec(dllimport)
#        define CPPUT_LIB_EXPORT __declspec(dllexport)
#        define CPPUT_LIB_LOCAL
#    else
#        if __GNUC__ >= 4
#            define CPPUT_LIB_IMPORT __attribute__((visibility("default")))
#            define CPPUT_LIB_EXPORT __attribute__((visibility("default")))
#            define CPPUT_LIB_LOCAL __attribute__((visibility("hidden")))
#        else
#            define CPPUT_LIB_IMPORT
#            define CPPUT_LIB_EXPORT
#            define CPPUT_LIB_LOCAL
#        endif
#    endif


#    ifdef CPPUT_COMPILE_SHARED_LIB
// compiled as a shared library (the default)
#        define CPPUT_LOCAL CPPUT_LIB_LOCAL

#        ifdef CPPUT_IMPORT_LIB
// this apparently makes sense only in WIN
#            define CPPUT_API CPPUT_LIB_IMPORT
#        else
#            define CPPUT_API CPPUT_LIB_EXPORT
#        endif
#    else
// compiled as a static library
#        define CPPUT_API
#        define CPPUT_LOCAL
#    endif

#endif
