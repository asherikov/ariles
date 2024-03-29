/**
    @file
    @author Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

/**
@defgroup yaml_cpp yaml-cpp
@ingroup config

@brief YAML serialization via https://github.com/jbeder/yaml-cpp.

@note @c yaml-cpp does not comply with the specification when it emits NaN's
and infinities, see https://github.com/jbeder/yaml-cpp/issues/507. @c ariles
includes a workaround for this issue.
*/


#pragma once

#define ARILES2_VISITOR_INCLUDED_yaml_cpp

#include <ariles2/internal/helpers.h>
#include <ariles2/visitors/config.h>


#include "./yaml_cpp/reader.h"
#include "./yaml_cpp/writer.h"


namespace ariles2
{
    /**
     * @brief YAML C++11 visitor.
     * @ingroup yaml_cpp
     */
    struct ARILES2_VISIBILITY_ATTRIBUTE yaml_cpp
    {
        using Reader = ariles2::cfgread::Visitor<ns_yaml_cpp::Reader>;
        using Writer = ariles2::cfgwrite::Visitor<ns_yaml_cpp::Writer>;
    };
}  // namespace ariles2
