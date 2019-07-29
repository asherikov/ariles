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

namespace ariles
{
    template <class t_Value>
    class PointerHandler<boost::shared_ptr<t_Value> >
    {
    public:
        typedef boost::shared_ptr<t_Value> Pointer;
        typedef t_Value Value;


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
}
#define ARILES_POINTER_TYPE                     boost::shared_ptr
#define ARILES_POINTER_HANDLER                  BoostSharedPtrHandler
#include <ariles/adapters/generic_pointer.h>


// this version is known to work
#if BOOST_VERSION >= 105800
#   include <boost/move/unique_ptr.hpp>
#   include <boost/move/make_unique.hpp>

namespace ariles
{
    template <class t_Value>
    class PointerHandler<boost::movelib::unique_ptr<t_Value> >
    {
    public:
        typedef boost::movelib::unique_ptr<t_Value> Pointer;
        typedef t_Value Value;


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
}

#   define ARILES_POINTER_TYPE      boost::movelib::unique_ptr
#   include <ariles/adapters/generic_pointer.h>
#endif
