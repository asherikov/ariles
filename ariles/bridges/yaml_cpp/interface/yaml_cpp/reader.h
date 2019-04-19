/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles
{
    namespace bridge
    {
        namespace yaml_cpp
        {
            /**
             * @brief Configuration reader class
             */
            class ARILES_VISIBILITY_ATTRIBUTE Reader :
                public ariles::bridge::yaml_cpp::Base<ariles::ReaderBase>
            {
                protected:
                    typedef ariles::Node<YAML::Node> NodeWrapper;


                protected:
                    /// instance of YAML parser
                    YAML::Parser  parser_;

                    /// Stack of nodes.
                    std::vector<NodeWrapper>    node_stack_;


                protected:
                    /**
                     * @brief Get current node
                     *
                     * @return pointer to the current node
                     */
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


                    std::size_t getMapSize(const bool /*expect_empty*/)
                    {
                        return (getRawNode().size());
                    }



                public:
                    /**
                     * @brief Constructor
                     *
                     * @param[in] file_name
                     */
                    explicit Reader(const std::string& file_name)
                    {
                        node_stack_.push_back(  NodeWrapper( YAML::LoadFile(file_name) )  );
                    }


                    /**
                     * @brief Constructor
                     *
                     * @param[in] input_stream
                     */
                    explicit Reader(std::istream& input_stream)
                    {
                        node_stack_.push_back(  NodeWrapper( YAML::Load(input_stream) )  );
                    }


                    bool descend(const std::string & child_name)
                    {
                        YAML::Node child = getRawNode()[child_name];

                        if (true == child.IsNull())
                        {
                            return(false);
                        }
                        else
                        {
                            node_stack_.push_back(NodeWrapper(child));
                            return(true);
                        }
                    }



                    void ascend()
                    {
                        node_stack_.pop_back();
                    }


                    bool getMapEntryNames(std::vector<std::string> &child_names)
                    {
                        YAML::Node selected_node = getRawNode();

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


                    std::size_t startArray()
                    {
                        ARILES_ASSERT(true == getRawNode().IsSequence(), "Entry is not an array.");

                        std::size_t size = getRawNode().size();
                        node_stack_.push_back(NodeWrapper(0, size));

                        return(size);
                    }


                    void shiftArray()
                    {
                        ARILES_ASSERT(true == node_stack_.back().isArray(),
                                      "Internal error: expected array.");
                        ARILES_ASSERT(node_stack_.back().index_ < node_stack_.back().size_,
                                      "Internal error: array has more elements than expected.");
                        ++node_stack_.back().index_;
                    }


                    void endArray()
                    {
                        node_stack_.pop_back();
                    }


                    #define ARILES_BASIC_TYPE(type) \
                        void readElement(type &element) \
                        { \
                            element = getRawNode().as<type>(); \
                        }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE
            };
        }
    }
}
