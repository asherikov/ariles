/**
    @file
    @author Alexander Sherikov
    @author Jan Michalczyk

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define ARILES2_VISITOR_INCLUDED_yaml_cpp03

#include <ariles2/internal/helpers.h>
#include <ariles2/visitors/config.h>


#include "./yaml_cpp03/reader.h"
#include "./yaml_cpp03/writer.h"


namespace ariles2
{
    /**
     * @brief YAML (C++03) visitor.
     */
    struct ARILES2_VISIBILITY_ATTRIBUTE yaml_cpp03
    {
        typedef ariles2::cfgread::Visitor<ns_yaml_cpp03::Reader> Reader;
        typedef ariles2::cfgwrite::Visitor<ns_yaml_cpp03::Writer> Writer;
    };
}  // namespace ariles2
