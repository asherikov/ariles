/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

/**
@defgroup ros2param ROS
@ingroup config

@brief ROS parameter server serialization.
*/


#pragma once

#define ARILES2_VISITOR_INCLUDED_ros2param

#include <ariles2/internal/helpers.h>
#include <ariles2/visitors/config.h>

#include <rclcpp/rclcpp.hpp>


#include "./ros2param/reader.h"
#include "./ros2param/writer.h"

namespace ariles2
{
    /**
     * @brief ROS parameter server visitor.
     * @ingroup ros2param
     */
    struct ARILES2_VISIBILITY_ATTRIBUTE ros2param
    {
        using Reader = ariles2::cfgread::Visitor<ns_ros2param::Reader>;
        using Writer = ariles2::cfgwrite::Visitor<ns_ros2param::Writer>;
    };
}  // namespace ariles2
