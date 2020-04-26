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

#include <ariles/visitors/yaml_cpp03.h>
#include "common.h"


namespace ariles
{
    namespace ns_yaml_cpp03
    {
        typedef ariles::Node<const YAML::Node *> NodeWrapper;
    }
}  // namespace ariles


namespace ariles
{
    namespace ns_yaml_cpp03
    {
        namespace impl
        {
            class Reader
            {
            public:
                /// instance of YAML parser
                YAML::Parser parser_;

                /// root node
                YAML::Node root_node_;

                /// Stack of nodes.
                std::vector<NodeWrapper> node_stack_;


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
}  // namespace ariles

namespace ariles
{
    namespace ns_yaml_cpp03
    {
        Reader::Reader(const std::string &file_name)
        {
            std::ifstream config_ifs;
            read::Visitor::openFile(config_ifs, file_name);

            impl_ = ImplPtr(new Impl());
            impl_->parser_.Load(config_ifs);
            impl_->parser_.GetNextDocument(impl_->root_node_);
            impl_->node_stack_.push_back(NodeWrapper(&impl_->root_node_));
        }


        Reader::Reader(std::istream &input_stream)
        {
            impl_ = ImplPtr(new Impl());
            impl_->parser_.Load(input_stream);
            impl_->parser_.GetNextDocument(impl_->root_node_);
            impl_->node_stack_.push_back(NodeWrapper(&impl_->root_node_));
        }



        std::size_t Reader::getMapSize(const bool /*expect_empty*/)
        {
            return (impl_->getRawNode().size());
        }



        bool Reader::descend(const std::string &child_name)
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



        void Reader::ascend()
        {
            impl_->node_stack_.pop_back();
        }


        bool Reader::getMapEntryNames(std::vector<std::string> &child_names)
        {
            const YAML::Node *selected_node = &impl_->getRawNode();

            if (YAML::NodeType::Map != selected_node->Type())
            {
                return (false);
            }
            else
            {
                child_names.resize(selected_node->size());

                std::size_t i = 0;
                for (YAML::Iterator it = selected_node->begin(); it != selected_node->end(); ++it, ++i)
                {
                    it.first() >> child_names[i];
                }
                return (true);
            }
        }


        std::size_t Reader::startArray()
        {
            std::size_t size = impl_->getRawNode().size();
            impl_->node_stack_.push_back(NodeWrapper(0, size));

            return (size);
        }


        void Reader::shiftArray()
        {
            ARILES_ASSERT(true == impl_->node_stack_.back().isArray(), "Internal error: expected array.");
            ARILES_ASSERT(
                    impl_->node_stack_.back().index_ < impl_->node_stack_.back().size_,
                    "Internal error: array has more elements than expected.");
            ++impl_->node_stack_.back().index_;
        }


        void Reader::endArray()
        {
            impl_->node_stack_.pop_back();
        }


#define ARILES_BASIC_TYPE(type)                                                                                        \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        impl_->getRawNode() >> element;                                                                                \
    }

        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

#undef ARILES_BASIC_TYPE
    }  // namespace ns_yaml_cpp03
}  // namespace ariles
