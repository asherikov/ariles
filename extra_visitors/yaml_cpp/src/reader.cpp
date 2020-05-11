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
        typedef ariles2::Node<YAML::Node> NodeWrapper;
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


            public:
                const YAML::Node getRawNode(const std::size_t depth)
                {
                    if (node_stack_[depth].isArray())
                    {
                        return (getRawNode(depth - 1)[node_stack_[depth].index_]);
                    }
                    else
                    {
                        return (node_stack_[depth].node_);
                    }
                }


                const YAML::Node getRawNode()
                {
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
            impl_ = ImplPtr(new Impl());
            impl_->node_stack_.push_back(NodeWrapper(YAML::LoadFile(file_name)));
        }


        Reader::Reader(std::istream &input_stream)
        {
            impl_ = ImplPtr(new Impl());
            impl_->node_stack_.push_back(NodeWrapper(YAML::Load(input_stream)));
        }



        std::size_t Reader::getMapSize(const bool /*expect_empty*/)
        {
            ARILES2_TRACE_FUNCTION;
            return (impl_->getRawNode().size());
        }


        bool Reader::descend(const std::string &child_name)
        {
            ARILES2_TRACE_FUNCTION;
            YAML::Node child = impl_->getRawNode()[child_name];

            if (false == child.IsDefined() or true == child.IsNull())
            {
                return (false);
            }
            else
            {
                impl_->node_stack_.push_back(NodeWrapper(child));
                return (true);
            }
        }



        void Reader::ascend()
        {
            ARILES2_TRACE_FUNCTION;
            impl_->node_stack_.pop_back();
        }


        bool Reader::getMapEntryNames(std::vector<std::string> &child_names)
        {
            ARILES2_TRACE_FUNCTION;
            YAML::Node selected_node = impl_->getRawNode();

            if (false == selected_node.IsMap())
            {
                return (false);
            }
            else
            {
                child_names.resize(selected_node.size());

                std::size_t i = 0;
                for (YAML::const_iterator it = selected_node.begin(); it != selected_node.end(); ++it, ++i)
                {
                    child_names[i] = it->first.as<std::string>();
                }
                return (true);
            }
        }


        std::size_t Reader::startArray()
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_ASSERT(true == impl_->getRawNode().IsSequence(), "Entry is not an array.");

            std::size_t size = impl_->getRawNode().size();
            impl_->node_stack_.push_back(NodeWrapper(0, size));

            return (size);
        }


        void Reader::shiftArray()
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_ASSERT(true == impl_->node_stack_.back().isArray(), "Internal error: expected array.");
            ARILES2_ASSERT(
                    impl_->node_stack_.back().index_ < impl_->node_stack_.back().size_,
                    "Internal error: array has more elements than expected.");
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
