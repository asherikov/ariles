/**
    @file
    @author Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
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
     */
    struct ARILES2_VISIBILITY_ATTRIBUTE yaml_cpp
    {
        typedef ariles2::cfgread::Visitor<ns_yaml_cpp::Reader> Reader;
        typedef ariles2::cfgwrite::Visitor<ns_yaml_cpp::Writer> Writer;
    };
}  // namespace ariles2
