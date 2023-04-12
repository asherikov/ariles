#pragma once


#include <string>
#include <vector>

#include "Vector3.h"
#include "Quaternion.h"

// NOLINTBEGIN(*)
namespace geometry_msgs
{
    template <class ContainerAllocator>
    struct Transform_
    {
        typedef Transform_<ContainerAllocator> Type;

        Transform_() : translation(), rotation()
        {
        }
        Transform_(const ContainerAllocator &_alloc) : translation(_alloc), rotation(_alloc)
        {
            (void)_alloc;
        }



        typedef ::geometry_msgs::Vector3_<ContainerAllocator> _translation_type;
        _translation_type translation;

        typedef ::geometry_msgs::Quaternion_<ContainerAllocator> _rotation_type;
        _rotation_type rotation;
    };  // struct Transform_

    typedef ::geometry_msgs::Transform_<std::allocator<void>> Transform;
}  // namespace geometry_msgs
// NOLINTEND(*)



namespace ariles_tests
{
    namespace rosmsg
    {
        class Transform : public ariles2::DefaultBase, public Eigen::Isometry3d
        {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, translation, Eigen::Vector3d)                                                              \
    ARILES2_TYPED_ENTRY_(v, rotation, Eigen::Quaterniond)
#include ARILES2_INITIALIZE

        public:
            virtual ~Transform()
            {
            }

#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                translation_.setRandom();
                rotation_.setIdentity();
            }
#endif
        };


        class TransformMember : public ariles2::DefaultBase
        {
#define ARILES2_ENTRIES(v) ARILES2_TYPED_ENTRY_(v, member, Eigen::Isometry3d)
#include ARILES2_INITIALIZE

        public:
            TransformMember()
            {
                member_.matrix().setZero();
            }

#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                member_.setIdentity();
            }
#endif
        };
    }  // namespace rosmsg
}  // namespace ariles_tests
