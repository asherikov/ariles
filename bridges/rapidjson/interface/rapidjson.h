/**
    @file
    @author Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


#include "../internal/helpers.h"
#include "../internal/reader_base.h"
#include "../internal/writer_base.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/istreamwrapper.h>

#ifdef ARILES_BRIDGE_jsonnet
extern "C" {
#include "libjsonnet.h"
}
#endif

#include "./rapidjson/reader.h"
#include "./rapidjson/writer.h"


#define ARILES_BRIDGE_INCLUDED_rapidjson


namespace ariles
{
    /**
     * @brief JSON bridge.
     */
    struct ARILES_VISIBILITY_ATTRIBUTE rapidjson : public BridgeSelectorBase
    {
        typedef bridge::rapidjson::Reader Reader;
        typedef bridge::rapidjson::Writer Writer;

#ifdef ARILES_BRIDGE_jsonnet
        struct ARILES_VISIBILITY_ATTRIBUTE jsonnet : public BridgeSelectorBase
        {
            typedef bridge::rapidjson::jsonnet::Reader Reader;
            typedef bridge::rapidjson::jsonnet::Writer Writer;
        };
#endif
    };
}
