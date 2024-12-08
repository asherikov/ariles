/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <ariles2/visitors/yaml_cpp.h>
#include <ariles2/visitors_impl/serialization.h>
#include <ariles2/visitors_impl/read.h>
#include <yaml-cpp/yaml.h>


namespace ariles2
{
    namespace ns_yaml_cpp
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_PRIVATE Node : public serialization::Node
            {
            public:
                const YAML::Node node_;
                YAML::const_iterator iterator_;

            public:
                template <class... t_Args>
                explicit Node(const YAML::Node &node, t_Args &&...args)
                  : serialization::Node(std::forward<t_Args>(args)...), node_(node)  // NOLINT
                {
                }
            };


            class ARILES2_VISIBILITY_PUBLIC Reader : public serialization::NodeStackBase<Node>,
                                                     public read::FileVisitorImplementation
            {
            public:
                template <class... t_Args>
                explicit Reader(t_Args &&...args) : read::FileVisitorImplementation(std::forward<t_Args>(args)...)
                {
                    emplace(YAML::Load(*input_streams_.back()));
                }


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
            makeImplPtr(file_name);
        }


        Reader::Reader(std::istream &input_stream)
        {
            makeImplPtr(input_stream);
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
                impl_->back().iterator_ = selected_node.begin();
                return (true);
            }
            return (false);
        }

        bool Reader::startIteratedMapElement(std::string &entry_name)
        {
            CPPUT_TRACE_FUNCTION;
            if (impl_->back().iterator_ != impl_->getRawNode().end())
            {
                entry_name = impl_->back().iterator_->first.as<std::string>();
                impl_->emplace(impl_->back().iterator_->second);
                return (true);
            }
            return (false);
        }

        void Reader::endIteratedMapElement()
        {
            CPPUT_TRACE_FUNCTION;
            impl_->pop();
            ++impl_->back().iterator_;
        }

        void Reader::endIteratedMap()
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_ASSERT(
                    impl_->back().iterator_ == impl_->getRawNode().end(), "End of iterated map has not been reached.");
        }


        std::size_t Reader::startArray()
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_ASSERT(impl_->getRawNode().IsSequence(), "Entry is not an array.");

            const std::size_t size = impl_->getRawNode().size();
            impl_->emplace(YAML::Node(), 0, size);

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
