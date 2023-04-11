/**
    @file
    @author Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

/**
@defgroup pugixml PugiXML
@ingroup config

@brief XML serialization via https://pugixml.org/.

@note Attributes are treated as childs while parsing and are never used for
emission.
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
     * @ingroup pugixml
     */
    struct ARILES2_VISIBILITY_ATTRIBUTE pugixml
    {
        using Reader = ariles2::cfgread::Visitor<ns_pugixml::Reader>;
        using Writer = ariles2::cfgwrite::Visitor<ns_pugixml::Writer>;
    };
}  // namespace ariles2
