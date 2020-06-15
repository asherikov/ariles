/**
    @file
    @author Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define ARILES2_VISITOR_INCLUDED_pugixml

#include <ariles2/internal/helpers.h>
#include <ariles2/visitors/config.h>


#include "./pugixml/reader.h"
#include "./pugixml/writer.h"


namespace ariles2
{
    /**
     * @brief pugixml visitor.
     */
    struct ARILES2_VISIBILITY_ATTRIBUTE pugixml
    {
        typedef ariles2::cfgread::Visitor<ns_pugixml::Reader> Reader;
        typedef ariles2::cfgwrite::Visitor<ns_pugixml::Writer> Writer;
    };
}  // namespace ariles2
