/**
    @file
    @author Jan Michalczyk
    @author Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <ariles2/visitors/yaml_cpp03.h>
#include "common.h"


namespace ariles2
{
    namespace ns_yaml_cpp03
    {
        typedef serialization::Node<const YAML::Node *> NodeWrapper;
    }
}  // namespace ariles2


namespace ariles2
{
    namespace ns_yaml_cpp03
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Reader
            {
            public:
                /// instance of YAML parser
                YAML::Parser parser_;

                /// root node
                YAML::Node root_node_;

                /// Stack of nodes.
                std::vector<NodeWrapper> node_stack_;
                std::vector<YAML::Iterator> iterator_stack_;


            public:
                const YAML::Node &getRawNode(const std::size_t depth)
                {
                    if (node_stack_[depth].isArray())
                    {
                        return (getRawNode(depth - 1)[node_stack_[depth].index_]);
                    }
                    else
                    {
                        return (*node_stack_[depth].node_);
                    }
                }


                const YAML::Node &getRawNode()
                {
                    return (getRawNode(node_stack_.size() - 1));
                }
            };
        }  // namespace impl
    }      // namespace ns_yaml_cpp03
}  // namespace ariles2

namespace ariles2
{
    namespace ns_yaml_cpp03
    {
        Reader::Reader(const std::string &file_name)
        {
            std::ifstream config_ifs;
            read::Visitor::openFile(config_ifs, file_name);

            impl_ = ImplPtr(new impl::Reader());
            impl_->parser_.Load(config_ifs);
            impl_->parser_.GetNextDocument(impl_->root_node_);
            impl_->node_stack_.push_back(NodeWrapper(&impl_->root_node_));
        }


        Reader::Reader(std::istream &input_stream)
        {
            impl_ = ImplPtr(new impl::Reader());
            impl_->parser_.Load(input_stream);
            impl_->parser_.GetNextDocument(impl_->root_node_);
            impl_->node_stack_.push_back(NodeWrapper(&impl_->root_node_));
        }



        void Reader::startMap(const SizeLimitEnforcementType limit_type, const std::size_t min, const std::size_t max)
        {
            ARILES2_TRACE_FUNCTION;
            checkSize(limit_type, impl_->getRawNode().size(), min, max);
        }

        bool Reader::startMapEntry(const std::string &child_name)
        {
            const YAML::Node *child = impl_->getRawNode().FindValue(child_name);

            if (child == NULL)
            {
                return (false);
            }
            else
            {
                impl_->node_stack_.push_back(NodeWrapper(child));
                return (true);
            }
        }

        void Reader::endMapEntry()
        {
            impl_->node_stack_.pop_back();
        }


        bool Reader::startIteratedMap(
                const SizeLimitEnforcementType limit_type,
                const std::size_t min,
                const std::size_t max)
        {
            ARILES2_TRACE_FUNCTION;
            checkSize(limit_type, impl_->getRawNode().size(), min, max);

            const YAML::Node *selected_node = &impl_->getRawNode();

            if (YAML::NodeType::Map == selected_node->Type())
            {
                impl_->iterator_stack_.push_back(selected_node->begin());
                return (true);
            }
            return (false);
        }

        bool Reader::startIteratedMapElement(std::string &entry_name)
        {
            if (impl_->iterator_stack_.back() != impl_->getRawNode().end())
            {
                impl_->node_stack_.push_back(&(impl_->iterator_stack_.back().second()));
                impl_->iterator_stack_.back().first() >> entry_name;
                return (true);
            }
            return (false);
        }

        void Reader::endIteratedMapElement()
        {
            ++impl_->iterator_stack_.back();
            impl_->node_stack_.pop_back();
        }

        void Reader::endIteratedMap()
        {
            ARILES2_ASSERT(
                    impl_->iterator_stack_.back() == impl_->getRawNode().end(),
                    "End of iterated map has not been reached.");
            impl_->iterator_stack_.pop_back();
        }


        std::size_t Reader::startArray()
        {
            std::size_t size = impl_->getRawNode().size();
            impl_->node_stack_.push_back(NodeWrapper(0, size));

            return (size);
        }

        void Reader::startArrayElement()
        {
            ARILES2_ASSERT(
                    impl_->node_stack_.back().index_ < impl_->node_stack_.back().size_,
                    "Internal error: array has more elements than expected.");
        }

        void Reader::endArrayElement()
        {
            ARILES2_ASSERT(true == impl_->node_stack_.back().isArray(), "Internal error: expected array.");
            ++impl_->node_stack_.back().index_;
        }

        void Reader::endArray()
        {
            impl_->node_stack_.pop_back();
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        impl_->getRawNode() >> element;                                                                                \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_yaml_cpp03
}  // namespace ariles2
