/**
    @file
    @author  Alexander Sherikov

    @copyright 2019 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Visibility defines, e.g., https://gcc.gnu.org/wiki/Visibility
*/

#pragma once

#ifndef H_ARILES_VISIBILITY
#    define H_ARILES_VISIBILITY

#    if defined _WIN32 || defined __CYGWIN__
#        define ARILES2_VISIBILITY_IMPORT __declspec(dllimport)
#        define ARILES2_VISIBILITY_PUBLIC __declspec(dllexport)
#        define ARILES2_VISIBILITY_PRIVATE
#    else
#        if __GNUC__ >= 4
#            define ARILES2_VISIBILITY_IMPORT __attribute__((visibility("default")))
#            define ARILES2_VISIBILITY_PUBLIC __attribute__((visibility("default")))
#            define ARILES2_VISIBILITY_PRIVATE __attribute__((visibility("hidden")))
#        else
#            define ARILES2_VISIBILITY_IMPORT
#            define ARILES2_VISIBILITY_PUBLIC
#            define ARILES2_VISIBILITY_PRIVATE
#        endif
#    endif


#    ifdef CPPUT_COMPILE_SHARED_LIB
// compiled as a shared library (the default)
#        define CPPUT_LOCAL ARILES2_VISIBILITY_PRIVATE

#        ifdef CPPUT_IMPORT_LIB
// this apparently makes sense only in WIN
#            define CPPUT_API ARILES2_VISIBILITY_IMPORT
#        else
#            define CPPUT_API ARILES2_VISIBILITY_PUBLIC
#        endif
#    else
// compiled as a static library
#        define CPPUT_API
#        define CPPUT_LOCAL
#    endif

#endif
