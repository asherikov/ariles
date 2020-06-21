/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

/**
@defgroup array Array

@brief Generates a vector of <std::string, double> pairs with flattened member names,
e.g., <"ariles_class.class_member.real_member", 3.4>.
*/


#pragma once

#define ARILES2_VISITOR_INCLUDED_array

#include <ariles2/internal/helpers.h>
#include <ariles2/visitors/config.h>

#include "./array/writer.h"

namespace ariles2
{
    /**
     * @brief Array visitor.
     * @ingroup array
     */
    struct ARILES2_VISIBILITY_ATTRIBUTE array
    {
        typedef ariles2::cfgwrite::Visitor<ns_array::Writer> Writer;
    };
}  // namespace ariles2
