/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <ariles/visitors/yaml_cpp.h>
#include <yaml-cpp/yaml.h>


namespace ariles
{
    namespace bridge
    {
        namespace yaml_cpp
        {
            typedef ariles::Node<YAML::Node> NodeWrapper;
        }
    }
}


namespace ariles
{
    namespace bridge
    {
        namespace yaml_cpp
        {
            namespace impl
            {
                class Reader
                {
                    public:
                        /// Stack of nodes.
                        std::vector<NodeWrapper>    node_stack_;


                    public:
                        const YAML::Node getRawNode(const std::size_t depth)
                        {
                            if (node_stack_[depth].isArray())
                            {
                                return(getRawNode(depth-1)[node_stack_[depth].index_]);
                            }
                            else
                            {
                                return(node_stack_[depth].node_);
                            }
                        }


                        const YAML::Node getRawNode()
                        {
                            return (getRawNode(node_stack_.size()-1));
                        }
                };
            }
        }
    }
}


namespace ariles
{
    namespace bridge
    {
        namespace yaml_cpp
        {
            Reader::Reader(const std::string& file_name)
            {
                impl_ = ImplPtr(new Impl());
                impl_->node_stack_.push_back(  NodeWrapper( YAML::LoadFile(file_name) )  );
            }


            Reader::Reader(std::istream& input_stream)
            {
                impl_ = ImplPtr(new Impl());
                impl_->node_stack_.push_back(  NodeWrapper( YAML::Load(input_stream) )  );
            }



            std::size_t Reader::getMapSize(const bool /*expect_empty*/)
            {
                return (impl_->getRawNode().size());
            }


            bool Reader::descend(const std::string & child_name)
            {
                YAML::Node child = impl_->getRawNode()[child_name];

                if (true == child.IsNull())
                {
                    return(false);
                }
                else
                {
                    impl_->node_stack_.push_back(NodeWrapper(child));
                    return(true);
                }
            }



            void Reader::ascend()
            {
                impl_->node_stack_.pop_back();
            }


            bool Reader::getMapEntryNames(std::vector<std::string> &child_names)
            {
                YAML::Node selected_node = impl_->getRawNode();

                if(false == selected_node.IsMap())
                {
                    return (false);
                }
                else
                {
                    child_names.resize(selected_node.size());

                    std::size_t i = 0;
                    for(YAML::const_iterator it = selected_node.begin(); it != selected_node.end(); ++it, ++i)
                    {
                        child_names[i] = it->first.as<std::string>();
                    }
                    return (true);
                }
            }


            std::size_t Reader::startArray()
            {
                ARILES_ASSERT(true == impl_->getRawNode().IsSequence(), "Entry is not an array.");

                std::size_t size = impl_->getRawNode().size();
                impl_->node_stack_.push_back(NodeWrapper(0, size));

                return(size);
            }


            void Reader::shiftArray()
            {
                ARILES_ASSERT(true == impl_->node_stack_.back().isArray(),
                              "Internal error: expected array.");
                ARILES_ASSERT(impl_->node_stack_.back().index_ < impl_->node_stack_.back().size_,
                              "Internal error: array has more elements than expected.");
                ++impl_->node_stack_.back().index_;
            }


            void Reader::endArray()
            {
                impl_->node_stack_.pop_back();
            }


            #define ARILES_BASIC_TYPE(type) \
                void Reader::readElement(type &element) \
                { \
                    element = impl_->getRawNode().as<type>(); \
                }

            ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

            #undef ARILES_BASIC_TYPE
        }
    }
}
