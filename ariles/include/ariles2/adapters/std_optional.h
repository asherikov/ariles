/**
    @file
    @author  Alexander Sherikov

    @copyright 2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <optional>
#include "../internal/helpers.h"

namespace ariles2
{
    template <class t_Value>
    class PointerHandler<std::optional<t_Value>>
    {
    public:
        using Pointer = std::optional<t_Value>;
        using Value = t_Value;


    public:
        static void allocate(Pointer &ptr)
        {
            ptr = t_Value();
        }

        static void reset(Pointer &ptr)
        {
            ptr.reset();
        }

        static bool isNull(const Pointer &ptr)
        {
            return (!ptr.has_value());
        }
    };
}  // namespace ariles2

#define ARILES2_POINTER_TYPE std::optional
#include <ariles2/adapters/generic_pointer.h>