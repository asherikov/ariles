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

// In old versions of RapidJSON it is impossible to specify flags
// as template parameter of PrettyWriter, so this is the only way
// to change them.
#define RAPIDJSON_WRITE_DEFAULT_FLAGS ::rapidjson::kWriteNanAndInfFlag
#define RAPIDJSON_PARSE_DEFAULT_FLAGS ::rapidjson::kParseNanAndInfFlag

#include <boost/lexical_cast.hpp>

#ifdef ARILES_BRIDGE_INCLUDED_jsonnet
extern "C" {
#include "libjsonnet.h"
}
#endif


#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>


namespace ariles
{
    namespace bridge
    {
        namespace rapidjson
        {
            template <class t_Base, class t_Node>
            class Base : public t_Base
            {
                protected:
                    typedef ariles::Node< t_Node * > NodeWrapper;


                protected:
                    /// instance of the parser
                    ::rapidjson::Document document_;

                    /// Stack of nodes.
                    std::vector<NodeWrapper>    node_stack_;


                protected:
                    /**
                     * @brief Get current node
                     *
                     * @return pointer to the current node
                     */
                    t_Node & getRawNode(const std::size_t depth)
                    {
                        if (node_stack_[depth].isArray())
                        {
                            return(getRawNode(depth-1)[node_stack_[depth].index_]);
                        }
                        else
                        {
                            return(*node_stack_[depth].node_);
                        }
                    }


                    t_Node & getRawNode()
                    {
                        if (true == node_stack_.empty())
                        {
                            return (document_);
                        }
                        else
                        {
                            return (getRawNode(node_stack_.size()-1));
                        }
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


#include "./rapidjson/istreamwrapper.h"
#include "./rapidjson/reader.h"
#include "./rapidjson/writer.h"


#define ARILES_BRIDGE_INCLUDED_rapidjson


namespace ariles
{
    /**
     * @brief JSON bridge.
     */
    struct ARILES_VISIBILITY_ATTRIBUTE rapidjson : public BridgeSelectorBase
    {
        typedef bridge::rapidjson::Reader Reader;
        typedef bridge::rapidjson::Writer Writer;

#ifdef ARILES_BRIDGE_INCLUDED_jsonnet
        struct ARILES_VISIBILITY_ATTRIBUTE jsonnet : public BridgeSelectorBase
        {
            typedef bridge::rapidjson::jsonnet::Reader Reader;
            typedef bridge::rapidjson::jsonnet::Writer Writer;
        };
#endif
    };
}
