/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


#include <ariles/internal/helpers.h>
#include <ariles/internal/node.h>
#include <ariles/visitors/config.h>

#include "./array/writer.h"


#define ARILES_BRIDGE_INCLUDED_array


namespace ariles
{
    /**
     * @brief Array bridge.
     */
    struct ARILES_VISIBILITY_ATTRIBUTE array
    {
        typedef ariles::cfgwrite::Visitor<bridge::array::Writer> Writer;
    };
}
