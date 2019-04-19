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


#include <boost/lexical_cast.hpp>
#include <pugixml.hpp>

namespace ariles
{
    namespace bridge
    {
        namespace pugixml
        {
            template <class t_Base>
            class Base : public t_Base
            {
                protected:
                    typedef ariles::Node< pugi::xml_node > NodeWrapper;


                protected:
                    pugi::xml_document document_;

                    std::vector<NodeWrapper>    node_stack_;


                protected:
                    /**
                     * @brief Get current node
                     *
                     * @return pointer to the current node
                     */
                    pugi::xml_node & getRawNode()
                    {
                        return(node_stack_.back().node_);
                    }


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


#include "./pugixml/reader.h"
#include "./pugixml/writer.h"


#define ARILES_BRIDGE_INCLUDED_pugixml


namespace ariles
{
    /**
     * @brief JSON bridge.
     */
    struct ARILES_VISIBILITY_ATTRIBUTE pugixml : public BridgeSelectorBase
    {
        typedef bridge::pugixml::Reader Reader;
        typedef bridge::pugixml::Writer Writer;
    };
}
