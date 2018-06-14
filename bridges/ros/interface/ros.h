/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#if defined(ARILES_ENABLED) || defined(ARILES_DISABLED)
    #error "All desired bridges must be included *before* the first inclusion of ariles.h, this bridge was not!"
#endif


#include "../internal/helpers.h"
#include "../internal/reader_base.h"
#include "../internal/writer_base.h"

#include <ros/ros.h>
// http://docs.ros.org/api/xmlrpcpp/html/classXmlRpc_1_1XmlRpcValue.html
#include <XmlRpcValue.h>

#include "./ros/reader.h"
#include "./ros/writer.h"


#define ARILES_ROS_NAMESPACE ros


// If something is stupid but it works, it is not stupid (c)
#ifndef ARILES_NAMESPACE_0
#   define ARILES_NAMESPACE_0 ARILES_ROS_NAMESPACE
#else
#   ifndef ARILES_NAMESPACE_1
#       define ARILES_NAMESPACE_1 ARILES_ROS_NAMESPACE
#   else
#       ifndef ARILES_NAMESPACE_2
#           define ARILES_NAMESPACE_2 ARILES_ROS_NAMESPACE
#       else
#           ifndef ARILES_NAMESPACE_3
#               define ARILES_NAMESPACE_3 ARILES_ROS_NAMESPACE
#           else
#               ifndef ARILES_NAMESPACE_4
#                   define ARILES_NAMESPACE_4 ARILES_ROS_NAMESPACE
#               else
#                   ifndef ARILES_NAMESPACE_5
#                       define ARILES_NAMESPACE_5 ARILES_ROS_NAMESPACE
#                   else
#                       ifndef ARILES_NAMESPACE_6
#                           define ARILES_NAMESPACE_6 ARILES_ROS_NAMESPACE
#                       else
#                           ifndef ARILES_NAMESPACE_7
#                               define ARILES_NAMESPACE_7 ARILES_ROS_NAMESPACE
#                           else
#                               error "Too many config namespaces."
#                           endif
#                       endif
#                   endif
#               endif
#           endif
#       endif
#   endif
#endif


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
