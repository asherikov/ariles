/**
    @file
    @author Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#if defined(ARILES_ENABLED) || defined(ARILES_DISABLED)
    #error "All desired bridges must be included *before* the first inclusion of ariles.h, this bridge was not!"
#endif


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


#define ARILES_RAPIDJSON_NAMESPACE rapidjson


// If something is stupid but it works, it is not stupid (c)
#ifndef ARILES_NAMESPACE_0
#   define ARILES_NAMESPACE_0 ARILES_RAPIDJSON_NAMESPACE
#else
#   ifndef ARILES_NAMESPACE_1
#       define ARILES_NAMESPACE_1 ARILES_RAPIDJSON_NAMESPACE
#   else
#       ifndef ARILES_NAMESPACE_2
#           define ARILES_NAMESPACE_2 ARILES_RAPIDJSON_NAMESPACE
#       else
#           ifndef ARILES_NAMESPACE_3
#               define ARILES_NAMESPACE_3 ARILES_RAPIDJSON_NAMESPACE
#           else
#               ifndef ARILES_NAMESPACE_4
#                   define ARILES_NAMESPACE_4 ARILES_RAPIDJSON_NAMESPACE
#               else
#                   ifndef ARILES_NAMESPACE_5
#                       define ARILES_NAMESPACE_5 ARILES_RAPIDJSON_NAMESPACE
#                   else
#                       ifndef ARILES_NAMESPACE_6
#                           define ARILES_NAMESPACE_6 ARILES_RAPIDJSON_NAMESPACE
#                       else
#                           ifndef ARILES_NAMESPACE_7
#                               define ARILES_NAMESPACE_7 ARILES_RAPIDJSON_NAMESPACE
#                           else
#                               error "Too many config namespaces."
#                           endif
#                       endif
#                   endif
#               endif
#           endif
#       endif
#   endif
#endif


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
