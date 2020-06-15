/**
    @file
    @author Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
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
     */
    struct ARILES2_VISIBILITY_ATTRIBUTE rapidjson
    {
        typedef ns_rapidjson::Reader ReaderBase;
        typedef ns_rapidjson::Writer WriterBase;

        typedef ariles2::cfgread::Visitor<ns_rapidjson::Reader> Reader;
        typedef ariles2::cfgwrite::Visitor<ns_rapidjson::Writer> Writer;
    };
}  // namespace ariles2
