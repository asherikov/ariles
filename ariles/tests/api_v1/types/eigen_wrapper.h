/**
    @file
    @author  Alexander Sherikov

    @copyright 2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles_tests
{
    struct Point :
#ifdef ARILES_ADAPTER_EIGEN
            public Eigen::Vector3d
#endif
    {
        /*
        geometry_msgs::Point getMsg() const
        {
            geometry_msgs::Point p;
            // convert *this to p
            return p;
        }
        */
    };


    struct ArilesPoint : public ariles::ConfigurableBase
    {
#define ARILES_SECTION_ID "ArilesPoint"
#define ARILES_AUTO_DEFAULTS
#define ARILES_ENTRIES ARILES_TYPED_ENTRY_(point, Point)
#include ARILES_INITIALIZE


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
        void randomize()
        {
#ifdef ARILES_ADAPTER_EIGEN
            point_.setZero();
#endif
        }
#endif
    };
}  // namespace ariles_tests
