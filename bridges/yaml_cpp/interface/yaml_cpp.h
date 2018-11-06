/**
    @file
    @author Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


#include <ariles/internal/helpers.h>
#include <ariles/internal/node.h>
#include <ariles/internal/reader_base.h>
#include <ariles/internal/writer_base.h>

#include <yaml-cpp/yaml.h>

namespace ariles
{
    namespace bridge
    {
        namespace yaml_cpp
        {
            template <class t_Base>
            class Base : public t_Base
            {
                public:
                    const BridgeFlags &getBridgeFlags() const
                    {
                        static BridgeFlags parameters(
                                BridgeFlags::SLOPPY_MAPS_SUPPORTED
                                | BridgeFlags::SLOPPY_PAIRS_SUPPORTED);
                        return (parameters);
                    }
            };
        }
    }
}


#include "./yaml_cpp/reader.h"
#include "./yaml_cpp/writer.h"


#define ARILES_BRIDGE_INCLUDED_yaml_cpp


namespace ariles
{
    /**
     * @brief YAML C++11 bridge.
     */
    struct ARILES_VISIBILITY_ATTRIBUTE yaml_cpp : public BridgeSelectorBase
    {
        typedef bridge::yaml_cpp::Reader Reader;
        typedef bridge::yaml_cpp::Writer Writer;
    };
}
