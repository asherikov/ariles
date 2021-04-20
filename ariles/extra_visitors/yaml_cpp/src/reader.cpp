/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <ariles2/visitors/yaml_cpp.h>
#include <yaml-cpp/yaml.h>


namespace ariles2
{
    namespace ns_yaml_cpp
    {
        typedef serialization::Node<YAML::Node> NodeWrapper;
    }
}  // namespace ariles2


namespace ariles2
{
    namespace ns_yaml_cpp
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Reader
            {
            public:
                /// Stack of nodes.
                std::vector<NodeWrapper> node_stack_;
                std::vector<YAML::const_iterator> iterator_stack_;


            public:
                const YAML::Node getRawNode(const std::size_t depth)
                {
                    ARILES2_TRACE_FUNCTION;
                    if (node_stack_[depth].isArray())
                    {
                        return (getRawNode(depth - 1)[node_stack_[depth].index_]);
                    }
                    return (node_stack_[depth].node_);
                }


                const YAML::Node getRawNode()
                {
                    ARILES2_TRACE_FUNCTION;
                    return (getRawNode(node_stack_.size() - 1));
                }
            };
        }  // namespace impl
    }      // namespace ns_yaml_cpp
}  // namespace ariles2


namespace ariles2
{
    namespace ns_yaml_cpp
    {
        Reader::Reader(const std::string &file_name)
        {
            impl_ = ImplPtr(new impl::Reader());
            impl_->node_stack_.push_back(NodeWrapper(YAML::LoadFile(file_name)));
        }


        Reader::Reader(std::istream &input_stream)
        {
            impl_ = ImplPtr(new impl::Reader());
            impl_->node_stack_.push_back(NodeWrapper(YAML::Load(input_stream)));
        }



        void Reader::startMap(const SizeLimitEnforcementType limit_type, const std::size_t min, const std::size_t max)
        {
            ARILES2_TRACE_FUNCTION;
            checkSize(limit_type, impl_->getRawNode().size(), min, max);
        }

        bool Reader::startMapEntry(const std::string &child_name)
        {
            ARILES2_TRACE_FUNCTION;
            YAML::Node child = impl_->getRawNode()[child_name];

            if (false == child.IsDefined() or true == child.IsNull())
            {
                return (false);
            }
            impl_->node_stack_.push_back(NodeWrapper(child));
            return (true);
        }

        void Reader::endMapEntry()
        {
            ARILES2_TRACE_FUNCTION;
            impl_->node_stack_.pop_back();
        }



        bool Reader::startIteratedMap(
                const SizeLimitEnforcementType limit_type,
                const std::size_t min,
                const std::size_t max)
        {
            ARILES2_TRACE_FUNCTION;
            checkSize(limit_type, impl_->getRawNode().size(), min, max);

            YAML::Node selected_node = impl_->getRawNode();

            if (true == selected_node.IsMap())
            {
                impl_->iterator_stack_.push_back(selected_node.begin());
                return (true);
            }
            return (false);
        }

        bool Reader::startIteratedMapElement(std::string &entry_name)
        {
            ARILES2_TRACE_FUNCTION;
            if (impl_->iterator_stack_.back() != impl_->getRawNode().end())
            {
                impl_->node_stack_.push_back(impl_->iterator_stack_.back()->second);
                entry_name = impl_->iterator_stack_.back()->first.as<std::string>();
                return (true);
            }
            return (false);
        }

        void Reader::endIteratedMapElement()
        {
            ARILES2_TRACE_FUNCTION;
            ++impl_->iterator_stack_.back();
            impl_->node_stack_.pop_back();
        }

        void Reader::endIteratedMap()
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_ASSERT(
                    impl_->iterator_stack_.back() == impl_->getRawNode().end(),
                    "End of iterated map has not been reached.");
            impl_->iterator_stack_.pop_back();
        }


        std::size_t Reader::startArray()
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_ASSERT(true == impl_->getRawNode().IsSequence(), "Entry is not an array.");

            std::size_t size = impl_->getRawNode().size();
            impl_->node_stack_.push_back(NodeWrapper(0, size));

            return (size);
        }


        void Reader::startArrayElement()
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_ASSERT(
                    impl_->node_stack_.back().index_ < impl_->node_stack_.back().size_,
                    "Internal error: namevalue.has more elements than expected.");
        }


        void Reader::endArrayElement()
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_ASSERT(true == impl_->node_stack_.back().isArray(), "Internal error: expected array.");
            ++impl_->node_stack_.back().index_;
        }


        void Reader::endArray()
        {
            ARILES2_TRACE_FUNCTION;
            impl_->node_stack_.pop_back();
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        element = impl_->getRawNode().as<type>();                                                                      \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_yaml_cpp
}  // namespace ariles2
