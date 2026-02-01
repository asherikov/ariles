/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


#include <boost/lexical_cast.hpp>
#include <nlohmann/json.hpp>

namespace ariles2
{
    namespace ns_nlohmann_json
    {
        template <class t_Node>
        class ARILES2_LOCAL ImplBase : public serialization::NodeStackBase<serialization::Node<t_Node *>>
        {
        public:
            using serialization::NodeStackBase<serialization::Node<t_Node *>>::node_stack_;

        public:
            /// instance of the JSON value
            ::nlohmann::ordered_json document_;


        public:
            /**
             * @brief Get current node
             *
             * @return reference to the current node
             */
            t_Node &getRawNode(const std::size_t depth)
            {
                if (node_stack_[depth].isArray())
                {
                    return (getRawNode(depth - 1)[node_stack_[depth].index_]);
                }

                return (*node_stack_[depth].node_);
            }


            const t_Node &getRawNode() const
            {
                if (node_stack_.empty())
                {
                    return (document_);
                }

                // Non-recursive implementation for const version
                const t_Node *current = node_stack_[0].node_;
                for (std::size_t i = 1; i < node_stack_.size(); ++i)
                {
                    if (node_stack_[i].isArray())
                    {
                        current = &((*current)[node_stack_[i].index_]);
                    }
                    else
                    {
                        current = node_stack_[i].node_;
                    }
                }
                return *current;
            }

            t_Node &getRawNode()
            {
                if (node_stack_.empty())
                {
                    return (document_);
                }

                return (getRawNode(node_stack_.size() - 1));
            }
        };
    }  // namespace ns_nlohmann_json
}  // namespace ariles2