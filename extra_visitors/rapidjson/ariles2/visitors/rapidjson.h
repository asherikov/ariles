/**
    @file
    @author Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

/**
@defgroup rapidjson RapidJSON
@ingroup config

@brief JSON serialization via https://rapidjson.org/.

@note NaN's and infinities, which are not allowed by \c JSON specification, are
optionally parsed / emitted using \c boost::lexical_cast.
*/


#pragma once

#define ARILES2_VISITOR_INCLUDED_rapidjson

#include <ariles2/internal/helpers.h>
#include <ariles2/visitors/config.h>


#include "./rapidjson/reader.h"
#include "./rapidjson/writer.h"


namespace ariles2
{
    /**
     * @brief JSON visitor.
     * @ingroup rapidjson
     */
    struct ARILES2_VISIBILITY_ATTRIBUTE rapidjson
    {
        using ReaderBase = ns_rapidjson::Reader;
        using WriterBase = ns_rapidjson::Writer;

        using Reader = ariles2::cfgread::Visitor<ns_rapidjson::Reader>;
        using Writer = ariles2::cfgwrite::Visitor<ns_rapidjson::Writer>;
    };
}  // namespace ariles2
