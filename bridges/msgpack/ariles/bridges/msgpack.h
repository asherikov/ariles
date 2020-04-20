/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#pragma once

#define ARILES_BRIDGE_INCLUDED_msgpack

#include <ariles/visitors/msgpack.h>

namespace ariles
{
    namespace bridge
    {
        namespace msgpack = ariles::ns_msgpack;
    }


    /**
     * @brief MessagePack bridge.
     */
    struct ARILES_VISIBILITY_ATTRIBUTE msgpack
    {
        typedef ariles::cfgread::Visitor<ns_msgpack::Reader> Reader;
        typedef ariles::cfgwrite::Visitor<ns_msgpack::Writer> Writer;

        struct ARILES_VISIBILITY_ATTRIBUTE compact
        {
            typedef ariles::cfgread::Visitor<ns_msgpack_compact::Reader> Reader;
            typedef ariles::cfgwrite::Visitor<ns_msgpack_compact::Writer> Writer;
        };
    };
}
