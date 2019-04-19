/**
    @file
    @author Jan Michalczyk
    @author Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles
{
    namespace bridge
    {
        namespace yaml_cpp03
        {
            /**
             * @brief Configuration reader class
             */
            class ARILES_VISIBILITY_ATTRIBUTE Reader : 
                public ariles::bridge::yaml_cpp03::Base<ariles::ReaderBase>
            {
                protected:
                    typedef ariles::Node<const YAML::Node *> NodeWrapper;


                protected:
                    /// instance of YAML parser
                    YAML::Parser  parser_;

                    /// root node
                    YAML::Node    root_node_;

                    /// Stack of nodes.
                    std::vector<NodeWrapper>    node_stack_;


                protected:
                    /**
                     * @brief Get current node
                     *
                     * @return pointer to the current node
                     */
                    const YAML::Node & getRawNode(const std::size_t depth)
                    {
                        if (node_stack_[depth].isArray())
                        {
                            return(getRawNode(depth-1)[node_stack_[depth].index_]);
                        }
                        else
                        {
                            return(*node_stack_[depth].node_);
                        }
                    }


                    const YAML::Node & getRawNode()
                    {
                        return(getRawNode(node_stack_.size()-1));
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
                        std::ifstream config_ifs;
                        ReaderBase::openFile(config_ifs, file_name);

                        parser_.Load(config_ifs),
                        parser_.GetNextDocument(root_node_);
                        node_stack_.push_back(NodeWrapper(&root_node_));
                    }


                    /**
                     * @brief Constructor
                     *
                     * @param[in] input_stream
                     */
                    explicit Reader(std::istream& input_stream)
                    {
                        parser_.Load(input_stream),
                        parser_.GetNextDocument(root_node_);
                        node_stack_.push_back(NodeWrapper(&root_node_));
                    }


                    bool descend(const std::string & child_name)
                    {
                        const YAML::Node * child = getRawNode().FindValue(child_name);

                        if (child == NULL)
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
                        const YAML::Node *selected_node = &getRawNode();

                        if(YAML::NodeType::Map != selected_node->Type())
                        {
                            return (false);
                        }
                        else
                        {
                            child_names.resize(selected_node->size());

                            std::size_t i = 0;
                            for(YAML::Iterator it = selected_node->begin(); it != selected_node->end(); ++it, ++i)
                            {
                                it.first() >> child_names[i];
                            }
                            return (true);
                        }
                    }


                    std::size_t startArray()
                    {
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
                            getRawNode() >> element; \
                        }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE
            };
        }
    }
}
