/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <boost/optional.hpp>

namespace
{
    template <class t_Value>
    class BoostOptionalHandler
    {
    public:
        typedef boost::optional<t_Value> PointerType;


    public:
        static void allocate(PointerType &ptr)
        {
            ptr = t_Value();
        }

        static void reset(PointerType &ptr)
        {
            ptr = boost::none;
        }

        static bool isNull(const PointerType &ptr)
        {
            return (boost::none == ptr);
        }
    };
}

#define ARILES_POINTER_TYPE                     boost::optional
#define ARILES_POINTER_HANDLER                  BoostOptionalHandler
#include <ariles/adapters/generic_pointer.h>
