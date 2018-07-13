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
            class ARILES_VISIBILITY_ATTRIBUTE Reader : public ariles::ReaderBase, public ariles::SloppyMapReaderBase
            {
                protected:
                    typedef ariles::Node<const YAML::Node> NodeWrapper;


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


                    /**
                     * @brief Default constructor
                     */
                    Reader()
                    {
                    }


                    /**
                     * @brief Descend to the entry with the given name
                     *
                     * @param[in] child_name child node name
                     *
                     * @return true if successful.
                     */
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


                    template<int t_size_limit_type>
                    std::size_t startMap(
                            const std::size_t & min = 0,
                            const std::size_t & max = 0)
                    {
                        return (checkSize<RelaxedSizeLimitType<t_size_limit_type>::value>(
                                    getRawNode().size(),
                                    min,
                                    max));
                    }


                    void endMap()
                    {
                    }


                    /**
                     * @brief Ascend from the current entry to its parent.
                     */
                    void ascend()
                    {
                        node_stack_.pop_back();
                    }


                    bool getEntryNames(std::vector<std::string> &child_names)
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
                        if (node_stack_.back().isArray())
                        {
                            ++node_stack_.back().index_;
                        }
                    }


                    void endArray()
                    {
                        node_stack_.pop_back();
                    }


                    template<class t_ElementType>
                        void readElement(t_ElementType &element)
                    {
                        getRawNode() >> element;
                    }
            };
        }
    }
}
