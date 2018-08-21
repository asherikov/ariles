/**
    @file
    @author Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


#include "../internal/helpers.h"
#include "../internal/node.h"
#include "../internal/reader_base.h"
#include "../internal/writer_base.h"


#include <pugixml.hpp>

#include "./pugixml/reader.h"
#include "./pugixml/writer.h"


#define ARILES_BRIDGE_INCLUDED_pugixml


namespace ariles
{
    /**
     * @brief JSON bridge.
     */
    struct ARILES_VISIBILITY_ATTRIBUTE pugixml : public BridgeSelectorBase
    {
        typedef bridge::pugixml::Reader Reader;
        typedef bridge::pugixml::Writer Writer;
    };
}
