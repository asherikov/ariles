/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


#include <ariles/visitors/ros.h>


// http://docs.ros.org/api/xmlrpcpp/html/classXmlRpc_1_1XmlRpcValue.html
#include <XmlRpcValue.h>


namespace ariles
{
    namespace bridge
    {
        namespace ros
        {
            typedef ariles::Node<XmlRpc::XmlRpcValue *> NodeWrapper;


            class ImplBase
            {
                public:
                    /// Stack of nodes.
                    std::vector<NodeWrapper>    node_stack_;

                    std::string                 root_name_;
                    XmlRpc::XmlRpcValue         root_value_;

                    ::ros::NodeHandle nh_;


                public:
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
            };
        }
    }
}
