/**
    @file
    @author  Alexander Sherikov
    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <memory>
#include "../internal/helpers.h"


namespace ariles2
{
    template <class t_Value>
    class ARILES2_VISIBILITY_ATTRIBUTE PointerHandler<std::shared_ptr<t_Value>>
    {
    public:
        using Pointer = std::shared_ptr<t_Value>;
        using Value = t_Value;


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
    class ARILES2_VISIBILITY_ATTRIBUTE PointerHandler<std::unique_ptr<t_Value>>
    {
    public:
        using Pointer = std::unique_ptr<t_Value>;
        using Value = t_Value;


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
}  // namespace ariles2

#define ARILES2_POINTER_TYPE std::shared_ptr
#include <ariles2/adapters/generic_pointer.h>


#define ARILES2_POINTER_TYPE std::unique_ptr
#include <ariles2/adapters/generic_pointer.h>
