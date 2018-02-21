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
    namespace ros
    {
        /**
         * @brief Configuration writer class
         */
        class ARILES_VISIBILITY_ATTRIBUTE Writer
        {
            protected:
                typedef ariles::Node<XmlRpc::XmlRpcValue> NodeWrapper;


            protected:
                /// Stack of nodes.
                std::vector<NodeWrapper>    node_stack_;

                std::string                 root_name_;
                XmlRpc::XmlRpcValue         root_value_;

                ::ros::NodeHandle nh_;



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
                        node_stack_.push_back(NodeWrapper( &( (*node_stack_.back().node_) [map_name]) ));
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
                void startMap(const std::size_t num_entries)
                {
                    //node_stack_.back().node_->setSize(num_entries);
                }


                /**
                 * @brief Ends a nested map in the configuration file
                 */
                void endMap()
                {
                }



                void startArray(const std::size_t size)
                {
                    node_stack_.back().node_->setSize(size);
                }

                void endArray()
                {
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
                    *node_stack_.back().node_ = element;
                }
        };
    }
}
