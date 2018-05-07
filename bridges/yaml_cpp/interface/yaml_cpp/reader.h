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
            class ARILES_VISIBILITY_ATTRIBUTE Reader : public ariles::ReaderBase, public ariles::SloppyMapReaderBase
            {
                protected:
                    class NodeWrapper
                    {
                        public:
                            YAML::Node          node_;
                            std::size_t         index_;
                            std::size_t         size_;
                            bool                is_array_;

                        public:
                            NodeWrapper(YAML::Node node) : node_(node)
                            {
                                index_ = 0;
                                size_ = 0;
                                is_array_ = false;
                            }

                            NodeWrapper(const std::size_t index, const std::size_t size) : index_(index)
                            {
                                size_ = size;
                                is_array_ = true;
                            }

                            bool isArray() const
                            {
                                return(is_array_);
                            }
                    };


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


                    /**
                     * @brief Ascend from the current entry to its parent.
                     */
                    void ascend()
                    {
                        node_stack_.pop_back();
                    }


                    bool getEntryNames(std::vector<std::string> &child_names)
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
                        element = getRawNode().as<t_ElementType>();
                    }
            };
        }
    }
}
