/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


// In old versions of RapidJSON it is impossible to specify flags
// as template parameter of PrettyWriter, so this is the only way
// to change them.
#define RAPIDJSON_WRITE_DEFAULT_FLAGS ::rapidjson::kWriteNanAndInfFlag
#define RAPIDJSON_PARSE_DEFAULT_FLAGS ::rapidjson::kParseNanAndInfFlag


#include <boost/lexical_cast.hpp>

#include <rapidjson/document.h>

#include "istreamwrapper.h"


namespace ariles2
{
    namespace ns_rapidjson
    {
        template <class t_Node>
        class ARILES2_LIB_LOCAL ImplBase
        {
        public:
            typedef serialization::Node<t_Node *> NodeWrapper;


        public:
            /// instance of the parser
            ::rapidjson::Document document_;

            /// Stack of nodes.
            std::vector<NodeWrapper> node_stack_;


        public:
            /**
             * @brief Get current node
             *
             * @return pointer to the current node
             */
            t_Node &getRawNode(const std::size_t depth)
            {
                if (node_stack_[depth].isArray())
                {
                    return (getRawNode(depth - 1)[node_stack_[depth].index_]);
                }

                return (*node_stack_[depth].node_);
            }


            t_Node &getRawNode()
            {
                if (true == node_stack_.empty())
                {
                    return (document_);
                }

                return (getRawNode(node_stack_.size() - 1));
            }
        };
    }  // namespace ns_rapidjson
}  // namespace ariles2
