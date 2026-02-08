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
        using NodeWrapper = serialization::Node<YAML::Node>;
    }
}  // namespace ariles2


namespace ariles2
{
    namespace ns_yaml_cpp
    {
        namespace impl
        {
            class Reader : public serialization::NodeStackBase<NodeWrapper>
            {
            public:
                std::vector<YAML::const_iterator> iterator_stack_;


            public:
                const YAML::Node getRawNode(const std::size_t depth)
                {
                    CPPUT_TRACE_FUNCTION;
                    if (node_stack_[depth].isArray())
                    {
                        return (getRawNode(depth - 1)[node_stack_[depth].index_]);
                    }
                    return (node_stack_[depth].node_);
                }


                const YAML::Node getRawNode()
                {
                    CPPUT_TRACE_FUNCTION;
                    return (getRawNode(node_stack_.size() - 1));
                }
            };
        }  // namespace impl
    }  // namespace ns_yaml_cpp
}  // namespace ariles2


namespace ariles2
{
    namespace ns_yaml_cpp
    {
        Reader::Reader(const std::string &file_name)
        {
            makeImplPtr();
            impl_->emplace(YAML::LoadFile(file_name));
        }


        Reader::Reader(std::istream &input_stream)
        {
            makeImplPtr();
            impl_->emplace(YAML::Load(input_stream));
        }



        void Reader::startMap(const SizeLimitEnforcementType limit_type, const std::size_t min, const std::size_t max)
        {
            CPPUT_TRACE_FUNCTION;
            checkSize(limit_type, impl_->getRawNode().size(), min, max);
        }

        bool Reader::startMapEntry(const std::string &child_name)
        {
            CPPUT_TRACE_FUNCTION;
            const YAML::Node child = impl_->getRawNode()[child_name];

            if (not child.IsDefined() or child.IsNull())
            {
                return (false);
            }
            impl_->emplace(child);
            return (true);
        }

        void Reader::endMapEntry()
        {
            CPPUT_TRACE_FUNCTION;
            impl_->pop();
        }



        bool Reader::startIteratedMap(
                const SizeLimitEnforcementType limit_type,
                const std::size_t min,
                const std::size_t max)
        {
            CPPUT_TRACE_FUNCTION;
            checkSize(limit_type, impl_->getRawNode().size(), min, max);

            YAML::Node selected_node = impl_->getRawNode();

            if (selected_node.IsMap())
            {
                impl_->iterator_stack_.emplace_back(selected_node.begin());
                return (true);
            }
            return (false);
        }

        bool Reader::startIteratedMapElement(std::string &entry_name)
        {
            CPPUT_TRACE_FUNCTION;
            if (impl_->iterator_stack_.back() != impl_->getRawNode().end())
            {
                impl_->emplace(impl_->iterator_stack_.back()->second);
                entry_name = impl_->iterator_stack_.back()->first.as<std::string>();
                return (true);
            }
            return (false);
        }

        void Reader::endIteratedMapElement()
        {
            CPPUT_TRACE_FUNCTION;
            ++impl_->iterator_stack_.back();
            impl_->pop();
        }

        void Reader::endIteratedMap()
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_ASSERT(
                    impl_->iterator_stack_.back() == impl_->getRawNode().end(),
                    "End of iterated map has not been reached.");
            impl_->iterator_stack_.pop_back();
        }


        std::size_t Reader::startArray()
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_ASSERT(impl_->getRawNode().IsSequence(), "Entry is not an array.");

            const std::size_t size = impl_->getRawNode().size();
            impl_->emplace(0, size);

            return (size);
        }


        void Reader::startArrayElement()
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_ASSERT(
                    impl_->back().index_ < impl_->back().size_,
                    "Internal error: array has more elements than expected.");
        }


        void Reader::endArrayElement()
        {
            impl_->shiftArray();
        }


        void Reader::endArray()
        {
            CPPUT_TRACE_FUNCTION;
            impl_->pop();
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        CPPUT_TRACE_FUNCTION;                                                                                          \
        element = impl_->getRawNode().as<type>();                                                                      \
    }

        CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_yaml_cpp
}  // namespace ariles2
