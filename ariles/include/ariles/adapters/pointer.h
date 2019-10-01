/**
    @file
    @author  Alexander Sherikov
    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once
#if __cplusplus >= 201103L

#include <memory>

namespace ariles
{
    template <class t_Value>
    class PointerHandler<std::shared_ptr<t_Value> >
    {
    public:
        typedef std::shared_ptr<t_Value> Pointer;
        typedef t_Value Value;


    public:
        static void allocate(Pointer &ptr)
        {
            ptr = std::make_shared<t_Value>();
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


    template <class t_Value>
    class PointerHandler<std::unique_ptr<t_Value> >
    {
    public:
        typedef std::unique_ptr<t_Value> Pointer;
        typedef t_Value Value;


    public:
        static void allocate(Pointer &ptr)
        {
            ptr.reset(new t_Value);
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

#define ARILES_POINTER_TYPE                             std::shared_ptr
#include <ariles/adapters/generic_pointer.h>


#define ARILES_POINTER_TYPE                             std::unique_ptr
#include <ariles/adapters/generic_pointer.h>

#endif
