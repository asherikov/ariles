/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#pragma once

#define ARILES_BRIDGE_INCLUDED_rapidjson

#include <ariles/visitors/rapidjson.h>


namespace ariles
{
    /**
     * @brief JSON bridge.
     */
    struct ARILES_VISIBILITY_ATTRIBUTE rapidjson
    {
        typedef ariles::bridge::rapidjson::Flags Flags;

        typedef ariles::cfgread::Visitor<bridge::rapidjson::Reader> Reader;
        typedef ariles::cfgwrite::Visitor<bridge::rapidjson::Writer> Writer;

#ifdef ARILES_VISITOR_INCLUDED_jsonnet
        struct ARILES_VISIBILITY_ATTRIBUTE jsonnet
        {
            typedef ariles::cfgread::Visitor<bridge::jsonnet::Reader<bridge::rapidjson::Reader> > Reader;
            typedef rapidjson::Writer Writer;
        };
#endif
    };
}
