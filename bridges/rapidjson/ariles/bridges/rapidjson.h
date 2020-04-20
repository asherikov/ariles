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
    namespace bridge
    {
        namespace rapidjson = ariles::ns_rapidjson;
    }


    /**
     * @brief JSON bridge.
     */
    struct ARILES_VISIBILITY_ATTRIBUTE rapidjson
    {
        typedef ariles::ns_rapidjson::Flags Flags;

        typedef ns_rapidjson::Reader ReaderBase;
        typedef ns_rapidjson::Writer WriterBase;

        typedef ariles::cfgread::Visitor<ns_rapidjson::Reader> Reader;
        typedef ariles::cfgwrite::Visitor<ns_rapidjson::Writer> Writer;

#ifdef ARILES_VISITOR_INCLUDED_jsonnet
        struct ARILES_VISIBILITY_ATTRIBUTE jsonnet
        {
            typedef ariles::cfgread::Visitor<ns_jsonnet::Reader<ns_rapidjson::Reader> > Reader;
            typedef rapidjson::Writer Writer;
        };
#endif
    };
}
