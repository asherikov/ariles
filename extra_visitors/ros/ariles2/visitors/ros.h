/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

/**
@defgroup ros ROS

@brief ROS parameter server serialization.
*/


#pragma once

#define ARILES2_VISITOR_INCLUDED_ros

#include <ariles2/internal/helpers.h>
#include <ariles2/visitors/config.h>

#include <ros/ros.h>


#include "./ros/reader.h"
#include "./ros/writer.h"

namespace ariles2
{
    /**
     * @brief ROS parameter server visitor.
     * @ingroup ros
     */
    struct ARILES2_VISIBILITY_ATTRIBUTE ros
    {
        typedef ariles2::cfgread::Visitor<ns_ros::Reader> Reader;
        typedef ariles2::cfgwrite::Visitor<ns_ros::Writer> Writer;
    };
}  // namespace ariles2
