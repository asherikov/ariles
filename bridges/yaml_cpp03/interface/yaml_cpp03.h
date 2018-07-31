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


#include "../internal/helpers.h"
#include "../internal/reader_base.h"
#include "../internal/writer_base.h"

#include ARILES_BRIDGE_yaml_cpp03_INCLUDE_HEADER

#include "./yaml_cpp03/reader.h"
#include "./yaml_cpp03/writer.h"

#define ARILES_BRIDGE_INCLUDED_yaml_cpp03


namespace ariles
{
    /**
     * @brief YAML bridge.
     */
    struct ARILES_VISIBILITY_ATTRIBUTE yaml_cpp03 : public BridgeSelectorBase
    {
        typedef bridge::yaml_cpp03::Reader Reader;
        typedef bridge::yaml_cpp03::Writer Writer;
    };
}
