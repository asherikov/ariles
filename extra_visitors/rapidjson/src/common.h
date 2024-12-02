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

#include <ariles2/visitors_impl/serialization.h>

#include <boost/lexical_cast.hpp>

#include <rapidjson/document.h>

#include "istreamwrapper.h"


namespace ariles2
{
    namespace ns_rapidjson
    {
        template <class t_Node>
        class CPPUT_LIB_LOCAL ImplBase : public serialization::NodeStackBase<t_Node>
        {
        public:
            using RawNode = decltype(t_Node::node_);

        public:
            /// instance of the parser
            ::rapidjson::Document document_;


        public:
            /**
             * @brief Get current node
             *
             * @return pointer to the current node
             */
            RawNode getRawNode(const std::size_t depth)
            {
                if (this->node_stack_[depth].isArray())
                {
                    return (&(*getRawNode(depth - 1))[this->node_stack_[depth].index_]);
                }

                return (this->node_stack_[depth].node_);
            }


            RawNode getRawNode()
            {
                if (this->node_stack_.empty())
                {
                    return (&document_);
                }

                return (getRawNode(this->node_stack_.size() - 1));
            }
        };
    }  // namespace ns_rapidjson
}  // namespace ariles2
