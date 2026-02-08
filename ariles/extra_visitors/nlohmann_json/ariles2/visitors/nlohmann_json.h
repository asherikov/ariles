/**
    @file
    @author Alexander Sherikov

    @copyright 2017-2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

/**
@defgroup nlohmann_json Nlohmann JSON
@ingroup config

@brief JSON serialization via https://github.com/nlohmann/json.

@note NaN's and infinities, which are not allowed by \c JSON specification, are
optionally parsed / emitted using \c boost::lexical_cast.
*/


#pragma once

#define ARILES2_VISITOR_INCLUDED_nlohmann_json

#include <ariles2/internal/helpers.h>
#include <ariles2/visitors/config.h>


#include "./nlohmann_json/reader.h"
#include "./nlohmann_json/writer.h"


namespace ariles2
{
    /**
     * @brief JSON visitor.
     * @ingroup nlohmann_json
     */
    struct nlohmann_json
    {
        using ReaderBase = ns_nlohmann_json::Reader;
        using WriterBase = ns_nlohmann_json::Writer;

        using Reader = ariles2::cfgread::Visitor<ns_nlohmann_json::Reader>;
        using Writer = ariles2::cfgwrite::Visitor<ns_nlohmann_json::Writer>;
    };
}  // namespace ariles2