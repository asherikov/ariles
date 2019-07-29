/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <boost/optional.hpp>

namespace ariles
{
    template <class t_Value>
    class PointerHandler<boost::optional<t_Value> >
    {
    public:
        typedef boost::optional<t_Value> Pointer;
        typedef t_Value Value;


    public:
        static void allocate(Pointer &ptr)
        {
            ptr = t_Value();
        }

        static void reset(Pointer &ptr)
        {
            ptr = boost::none;
        }

        static bool isNull(const Pointer &ptr)
        {
            return (boost::none == ptr);
        }
    };
}

#define ARILES_POINTER_TYPE                     boost::optional
#include <ariles/adapters/generic_pointer.h>
