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
        namespace ros
        {
            /**
             * @brief Configuration writer class
             */
            class ARILES_VISIBILITY_ATTRIBUTE Writer : public ariles::WriterBase, public ariles::SloppyMapWriterBase
            {
                protected:
                    typedef ariles::Node<XmlRpc::XmlRpcValue> NodeWrapper;


                protected:
                    /// Stack of nodes.
                    std::vector<NodeWrapper>    node_stack_;

                    std::string                 root_name_;
                    XmlRpc::XmlRpcValue         root_value_;

                    ::ros::NodeHandle nh_;


                protected:
                    /**
                     * @brief Get current node
                     *
                     * @return pointer to the current node
                     */
                    XmlRpc::XmlRpcValue & getRawNode(const std::size_t depth)
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


                    XmlRpc::XmlRpcValue & getRawNode()
                    {
                        return(getRawNode(node_stack_.size()-1));
                    }


                public:
                    explicit Writer(const ::ros::NodeHandle &nh)
                    {
                        nh_ = nh;
                    }


                    ~Writer()
                    {
                    }



                    /**
                     * @brief Starts a nested map in the configuration file
                     */
                    void initRoot()
                    {
                        root_name_ = "";
                        root_value_.clear();
                    }


                    /**
                     * @brief Flush the configuration to the file
                     */
                    void flush()
                    {
                        nh_.setParam(root_name_, root_value_);
                        root_name_.clear();
                    }



                    /**
                     * @brief Starts a nested map in the configuration file
                     *
                     * @param[in] map_name name of the map
                     */
                    void descend(const std::string &map_name)
                    {
                        if (0 == node_stack_.size())
                        {
                            root_name_ = map_name;
                            node_stack_.push_back(&root_value_);
                        }
                        else
                        {
                            node_stack_.push_back(   NodeWrapper(  &( getRawNode()[map_name] )  )   );
                        }
                    }


                    void ascend()
                    {
                        node_stack_.pop_back();
                    }



                    /**
                     * @brief Starts a nested map in the configuration file
                     *
                     * @param[in] num_entries number of child entries
                     */
                    void startMap(const std::size_t /*num_entries*/)
                    {
                    }


                    /**
                     * @brief Ends a nested map in the configuration file
                     */
                    void endMap()
                    {
                    }


                    void startArray(const std::size_t size, const bool /*compact*/ = false)
                    {
                        getRawNode().setSize(size);
                        node_stack_.push_back(NodeWrapper(0, size));
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


                    /**
                     * @brief Write a configuration entry (scalar template)
                     *
                     * @tparam t_EntryType type of the entry
                     *
                     * @param[in] entry      data
                     */
                    template<class t_Element>
                        void writeElement(const t_Element & element)
                    {
                        getRawNode() = element;
                    }


                    /**
                     * @brief Write a configuration entry (scalar template)
                     *
                     * @param[in] entry      data
                     */
                    void writeElement(const long int element)
                    {
                        ARILES_ASSERT(  (element <= std::numeric_limits<int>::max())
                                        && (element >= std::numeric_limits<int>::min()),
                                        "Integer is too large to be saved.");

                        getRawNode() = static_cast<int>(element);
                    }


                    /**
                     * @brief Write a configuration entry ()
                     *
                     * @param[in] entry      data
                     */
                    void writeElement(const long unsigned int element)
                    {
                        ARILES_ASSERT(  element <= std::numeric_limits<int>::max(),
                                        "Integer is too large to be saved.");

                        getRawNode() = static_cast<int>(element);
                    }


                    /**
                     * @brief Write a configuration entry ()
                     *
                     * @param[in] entry      data
                     */
                    void writeElement(const unsigned int element)
                    {
                        ARILES_ASSERT(  element <= std::numeric_limits<int>::max(),
                                        "Integer is too large to be saved.");

                        getRawNode() = static_cast<int>(element);
                    }
            };
        }
    }
}
