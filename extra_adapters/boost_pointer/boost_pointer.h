/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/smart_ptr/make_shared.hpp>
#include "../internal/helpers.h"

namespace ariles2
{
    template <class t_Value>
    class PointerHandler<boost::shared_ptr<t_Value>>
    {
    public:
        using Pointer = boost::shared_ptr<t_Value>;
        using Value = t_Value;


    public:
        static void allocate(Pointer &ptr)
        {
            ptr = boost::make_shared<t_Value>();
        }

        static void reset(Pointer &ptr)
        {
            ptr.reset();
        }

        static bool isNull(const Pointer &ptr)
        {
            return (NULL == ptr);
        }
    };
}  // namespace ariles2
#define ARILES2_POINTER_TYPE boost::shared_ptr
#define ARILES2_POINTER_HANDLER BoostSharedPtrHandler
#include <ariles2/adapters/generic_pointer.h>


// this version is known to work
#if BOOST_VERSION >= 105800
#    include <boost/move/unique_ptr.hpp>
#    include <boost/move/make_unique.hpp>

namespace ariles2
{
    template <class t_Value>
    class PointerHandler<boost::movelib::unique_ptr<t_Value>>
    {
    public:
        using Pointer = boost::movelib::unique_ptr<t_Value>;
        using Value = t_Value;


    public:
        static void allocate(Pointer &ptr)
        {
            ptr = boost::movelib::make_unique<t_Value>();
        }

        static void reset(Pointer &ptr)
        {
            ptr.reset();
        }

        static bool isNull(const Pointer &ptr)
        {
            return (NULL == ptr);
        }
    };
}  // namespace ariles2

#    define ARILES2_POINTER_TYPE boost::movelib::unique_ptr
#    include <ariles2/adapters/generic_pointer.h>
#endif
