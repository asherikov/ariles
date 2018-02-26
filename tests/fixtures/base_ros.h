/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <ros/ros.h>

class FixtureBase
{
    public:
        ros::NodeHandle *nh_;


    public:
        FixtureBase()
        {
            int argn = 0;
            ros::init(argn, NULL, "FixtureBase");

            nh_ = new ros::NodeHandle();
        }


        ~FixtureBase()
        {
            delete nh_;
        }


        ros::NodeHandle & getInitializer(const std::string & string_id)
        {
            return (*nh_);
        }
};
