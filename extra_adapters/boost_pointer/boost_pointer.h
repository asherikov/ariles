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
    class BoostSharedPtrHandler
    {
    public:
        typedef boost::shared_ptr<t_Value> PointerType;


    public:
        static void allocate(PointerType &ptr)
        {
            ptr = boost::make_shared<t_Value>();
        }

        static void reset(PointerType &ptr)
        {
            ptr.reset();
        }

        static bool isNull(const PointerType &ptr)
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
    class BoostUniquePtrHandler
    {
    public:
        typedef boost::movelib::unique_ptr<t_Value> PointerType;


    public:
        static void allocate(PointerType &ptr)
        {
            ptr = boost::movelib::make_unique<t_Value>();
        }

        static void reset(PointerType &ptr)
        {
            ptr.reset();
        }

        static bool isNull(const PointerType &ptr)
        {
            return (NULL == ptr);
        }
    };
}

#   define ARILES_POINTER_TYPE      boost::movelib::unique_ptr
#   define ARILES_POINTER_HANDLER   BoostUniquePtrHandler
#   include <ariles/adapters/generic_pointer.h>
#endif
