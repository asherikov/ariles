/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define ARILES_VISITOR_INCLUDED_array

#include <ariles2/internal/helpers.h>
#include <ariles2/internal/node.h>
#include <ariles2/visitors/config.h>

#include "./array/writer.h"


namespace ariles
{
    /**
     * @brief Array visitor.
     */
    struct ARILES_VISIBILITY_ATTRIBUTE array
    {
        typedef ariles::cfgwrite::Visitor<ns_array::Writer> Writer;
    };
}  // namespace ariles
