#pragma once


#include <string>
#include <vector>

#include "Vector3.h"

// NOLINTBEGIN(*)
namespace geometry_msgs
{
    template <class ContainerAllocator>
    struct Twist_
    {
        typedef Twist_<ContainerAllocator> Type;

        Twist_() : linear(), angular()
        {
        }
        Twist_(const ContainerAllocator &_alloc) : linear(_alloc), angular(_alloc)
        {
            (void)_alloc;
        }



        typedef ::geometry_msgs::Vector3_<ContainerAllocator> _linear_type;
        _linear_type linear;

        typedef ::geometry_msgs::Vector3_<ContainerAllocator> _angular_type;
        _angular_type angular;
    };  // struct Twist_

    typedef ::geometry_msgs::Twist_<std::allocator<void>> Twist;
}  // namespace geometry_msgs
// NOLINTEND(*)



namespace ariles_tests
{
    namespace rosmsg
    {
        class Twist : public ariles2::DefaultBase
        {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, linear, Eigen::Vector3d)                                                                   \
    ARILES2_TYPED_ENTRY_(v, angular, Eigen::Vector3d)
#include ARILES2_INITIALIZE

        public:
            virtual ~Twist()
            {
            }

#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                linear_.setRandom();
                angular_.setIdentity();
            }
#endif
        };
    }  // namespace rosmsg
}  // namespace ariles_tests
