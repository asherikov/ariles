/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


#include "../internal/helpers.h"
#include "../internal/reader_base.h"
#include "../internal/writer_base.h"

#include <ros/ros.h>
// http://docs.ros.org/api/xmlrpcpp/html/classXmlRpc_1_1XmlRpcValue.html
#include <XmlRpcValue.h>

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
