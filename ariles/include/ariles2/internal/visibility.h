/**
    @file
    @author  Alexander Sherikov

    @copyright 2019 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Visibility defines, e.g., https://gcc.gnu.org/wiki/Visibility
*/

#pragma once

#ifndef H_ARILES2_VISIBILITY
#    define H_ARILES2_VISIBILITY

#    ifndef ARILES2_LOCAL
#        if defined _WIN32 || defined __CYGWIN__
#            define ARILES2_LIB_IMPORT __declspec(dllimport)
#            define ARILES2_LIB_EXPORT __declspec(dllexport)
#            define ARILES2_LIB_LOCAL
#        else
#            if __GNUC__ >= 4
#                define ARILES2_LIB_IMPORT __attribute__((visibility("default")))
#                define ARILES2_LIB_EXPORT __attribute__((visibility("default")))
#                define ARILES2_LIB_LOCAL __attribute__((visibility("hidden")))
#            else
#                define ARILES2_LIB_IMPORT
#                define ARILES2_LIB_EXPORT
#                define ARILES2_LIB_LOCAL
#            endif
#        endif


#        ifdef ARILES2_COMPILE_SHARED_LIB
// compiled as a shared library (the default)
#            define ARILES2_LOCAL ARILES2_LIB_LOCAL

#            ifdef ARILES2_IMPORT_LIB
// this apparently makes sense only in WIN
#                define ARILES2_API ARILES2_LIB_IMPORT
#            else
#                define ARILES2_API ARILES2_LIB_EXPORT
#            endif
#        else
// compiled as a static library
#            define ARILES2_API
#            define ARILES2_LOCAL
#        endif
#    endif
#endif
