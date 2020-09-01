#pragma once


#include <string>
#include <vector>

#include "Transform.h"
#include "Twist.h"

namespace trajectory_msgs
{
    template <class ContainerAllocator>
    struct MultiDOFJointTrajectoryPoint_
    {
        typedef MultiDOFJointTrajectoryPoint_<ContainerAllocator> Type;

        MultiDOFJointTrajectoryPoint_() : transforms(), velocities(), accelerations(), time_from_start()
        {
        }
        MultiDOFJointTrajectoryPoint_(const ContainerAllocator &_alloc)
          : transforms(_alloc), velocities(_alloc), accelerations(_alloc), time_from_start()
        {
            (void)_alloc;
        }



        typedef std::vector<
                ::geometry_msgs::Transform_<ContainerAllocator>,
                typename ContainerAllocator::template rebind< ::geometry_msgs::Transform_<ContainerAllocator> >::other>
                _transforms_type;
        _transforms_type transforms;

        typedef std::vector<
                ::geometry_msgs::Twist_<ContainerAllocator>,
                typename ContainerAllocator::template rebind< ::geometry_msgs::Twist_<ContainerAllocator> >::other>
                _velocities_type;
        _velocities_type velocities;

        typedef std::vector<
                ::geometry_msgs::Twist_<ContainerAllocator>,
                typename ContainerAllocator::template rebind< ::geometry_msgs::Twist_<ContainerAllocator> >::other>
                _accelerations_type;
        _accelerations_type accelerations;

        typedef uint64_t _time_from_start_type;
        _time_from_start_type time_from_start;
    };  // struct MultiDOFJointTrajectoryPoint_

    typedef ::trajectory_msgs::MultiDOFJointTrajectoryPoint_<std::allocator<void> > MultiDOFJointTrajectoryPoint;
}  // namespace trajectory_msgs



namespace ariles_tests
{
    namespace rosmsg
    {
        class MultiDOFJointTrajectoryPoint : public ariles2::DefaultBase
        {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, transforms, std::vector<ariles_tests::rosmsg::Transform>)                                  \
    ARILES2_TYPED_ENTRY_(v, velocities, std::vector<ariles_tests::rosmsg::Twist>)                                      \
    ARILES2_TYPED_ENTRY_(v, accelerations, std::vector<ariles_tests::rosmsg::Twist>)                                   \
    ARILES2_TYPED_ENTRY_(v, time_from_start, std::size_t)
#include ARILES2_INITIALIZE

        public:
            virtual ~MultiDOFJointTrajectoryPoint()
            {
            }

#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                boost::random::random_device random_generator;

                transforms_.resize(5);
                velocities_.resize(5);
                accelerations_.resize(5);

                for (std::size_t i = 0; i < 5; ++i)
                {
                    ariles_tests::rosmsg::Transform transform;
                    ariles_tests::rosmsg::Twist twist;

                    transform.randomize();
                    twist.randomize();

                    transforms_.push_back(transform);
                    velocities_.push_back(twist);

                    twist.randomize();
                    accelerations_.push_back(twist);
                }

                time_from_start_ = GET_RANDOM_INT;
            }
#endif
        };
    }  // namespace rosmsg
}  // namespace ariles_tests
