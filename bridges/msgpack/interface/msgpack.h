/**
    @file
    @author Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


#include <ariles/internal/helpers.h>
#include <ariles/internal/node.h>
#include <ariles/visitors/config.h>

#include <msgpack.hpp>


namespace ariles
{
    namespace bridge
    {
        namespace msgpack
        {
            template <class t_Base>
            class Base : public t_Base
            {
                public:
                    const BridgeFlags &getBridgeFlags() const
                    {
                        static BridgeFlags parameters; // all disabled
                        return (parameters);
                    }
            };
        }
    }
}

#include <boost/smart_ptr/shared_ptr.hpp>

#include "./msgpack/reader.h"
#include "./msgpack/writer.h"
#include "./msgpack/reader_compact.h"
#include "./msgpack/writer_compact.h"


#define ARILES_BRIDGE_INCLUDED_msgpack


namespace ariles
{
    /**
     * @brief MessagePack bridge.
     */
    struct ARILES_VISIBILITY_ATTRIBUTE msgpack
    {
        typedef ariles::cfgread::Visitor<bridge::msgpack::Reader> Reader;
        typedef ariles::cfgwrite::Visitor<bridge::msgpack::Writer> Writer;

        struct ARILES_VISIBILITY_ATTRIBUTE compact
        {
            typedef ariles::cfgread::Visitor<bridge::msgpack::compact::Reader> Reader;
            typedef ariles::cfgwrite::Visitor<bridge::msgpack::compact::Writer> Writer;
        };
    };
}
