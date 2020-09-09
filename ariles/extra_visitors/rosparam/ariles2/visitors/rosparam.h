/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

/**
@defgroup rosparam ROS
@ingroup config

@brief ROS parameter server serialization.
*/


#pragma once

#define ARILES2_VISITOR_INCLUDED_rosparam

#include <ariles2/internal/helpers.h>
#include <ariles2/visitors/config.h>

#include <ros/ros.h>


#include "./rosparam/reader.h"
#include "./rosparam/writer.h"

namespace ariles2
{
    /**
     * @brief ROS parameter server visitor.
     * @ingroup rosparam
     */
    struct ARILES2_VISIBILITY_ATTRIBUTE rosparam
    {
        typedef ariles2::cfgread::Visitor<ns_rosparam::Reader> Reader;
        typedef ariles2::cfgwrite::Visitor<ns_rosparam::Writer> Writer;
    };
}  // namespace ariles2
