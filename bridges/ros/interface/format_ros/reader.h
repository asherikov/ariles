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
         * @brief Configuration reader class
         */
        class ARILES_VISIBILITY_ATTRIBUTE Reader : public ariles::ReaderBase
        {
            protected:
                typedef ariles::Node<std::string> NodeWrapper;


            protected:
                /// Stack of nodes.
                std::vector<NodeWrapper>    node_stack_;


            protected:

                // http://docs.ros.org/api/xmlrpcpp/html/classXmlRpc_1_1XmlRpcValue.html
/*
XmlRpc::XmlRpcValue my_list;
nh.getParam("my_list", my_list);
ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

for (int32_t i = 0; i < my_list.size(); ++i)
{
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    sum += static_cast<double>(my_list[i]);
}
*/

                /*
                 * @brief Get current node
                 *
                 * @return pointer to the current node
                 *
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
                */



            public:
                /**
                 * @brief Constructor
                 *
                 * @param[in] file_name
                 */
                explicit Reader(const std::string& file_name)
                {
                    //openFile(file_name);
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
                    /*
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
                    */
                    return(true);
                }


                /**
                 * @brief Ascend from the current entry to its parent.
                 */
                void ascend()
                {
                    node_stack_.pop_back();
                }


                std::size_t startArray()
                {
                    //std::size_t size = getRawNode().size();
                    std::size_t size = 0;
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
                    //getRawNode() >> element;
                }
        };
    }
}
