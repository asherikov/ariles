/**
    @file
    @author Alexander Sherikov

    @copyright 2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define ARILES2_VISITOR_INCLUDED_graphviz

#include <ariles2/internal/helpers.h>
#include <ariles2/visitors/config.h>

#include "./graphviz/writer.h"

namespace ariles2
{
    /**
     * @brief Graphviz visitor.
     */
    struct ARILES2_VISIBILITY_ATTRIBUTE graphviz
    {
        typedef ariles2::cfgwrite::Visitor<ns_graphviz::Writer> Writer;
    };
}  // namespace ariles2
