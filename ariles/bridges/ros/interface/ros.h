/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


#include <ariles/internal/helpers.h>
#include <ariles/internal/node.h>
#include <ariles/internal/reader_base.h>
#include <ariles/internal/writer_base.h>

#include <ros/ros.h>
// http://docs.ros.org/api/xmlrpcpp/html/classXmlRpc_1_1XmlRpcValue.html
#include <XmlRpcValue.h>


namespace ariles
{
    namespace bridge
    {
        namespace ros
        {
            template <class t_Base>
            class Base : public t_Base
            {
                protected:
                    typedef ariles::Node<XmlRpc::XmlRpcValue *> NodeWrapper;


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
                    const BridgeFlags &getBridgeFlags() const
                    {
                        static BridgeFlags parameters(
                                BridgeFlags::SLOPPY_MAPS_SUPPORTED
                                | BridgeFlags::SLOPPY_PAIRS_SUPPORTED);
                        return (parameters);
                    }
            };
        }
    }
}


#include "./ros/reader.h"
#include "./ros/writer.h"

#define ARILES_BRIDGE_INCLUDED_ros

namespace ariles
{
    /**
     * @brief ROS parameter server bridge.
     */
    struct ARILES_VISIBILITY_ATTRIBUTE ros : public BridgeSelectorBase
    {
        typedef bridge::ros::Reader Reader;
        typedef bridge::ros::Writer Writer;
    };
}
