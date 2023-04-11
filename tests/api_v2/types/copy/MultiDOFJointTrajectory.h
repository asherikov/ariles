#pragma once


#include <string>
#include <vector>

#include "Header.h"
#include "MultiDOFJointTrajectoryPoint.h"

// NOLINTBEGIN(*)
namespace trajectory_msgs
{
    template <class ContainerAllocator>
    struct MultiDOFJointTrajectory_
    {
        typedef MultiDOFJointTrajectory_<ContainerAllocator> Type;

        MultiDOFJointTrajectory_() : header(), joint_names(), points()
        {
        }
        MultiDOFJointTrajectory_(const ContainerAllocator &_alloc) : header(_alloc), joint_names(_alloc), points(_alloc)
        {
            (void)_alloc;
        }



        typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
        _header_type header;

        typedef std::vector<
                std::basic_string<
                        char,
                        std::char_traits<char>,
                        typename ContainerAllocator::template rebind<char>::other>,
                typename ContainerAllocator::template rebind<std::basic_string<
                        char,
                        std::char_traits<char>,
                        typename ContainerAllocator::template rebind<char>::other>>::other>
                _joint_names_type;
        _joint_names_type joint_names;

        typedef std::vector<
                ::trajectory_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator>,
                typename ContainerAllocator::template rebind<
                        ::trajectory_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator>>::other>
                _points_type;
        _points_type points;
    };  // struct MultiDOFJointTrajectory_

    typedef ::trajectory_msgs::MultiDOFJointTrajectory_<std::allocator<void>> MultiDOFJointTrajectory;
}  // namespace trajectory_msgs
// NOLINTEND(*)



namespace ariles_tests
{
    namespace rosmsg
    {
        class MultiDOFJointTrajectory : public ariles2::DefaultBase
        {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, points, std::vector<ariles_tests::rosmsg::MultiDOFJointTrajectoryPoint>)                   \
    ARILES2_TYPED_ENTRY_(v, joint_names, std::vector<std::string>)
#include ARILES2_INITIALIZE

        public:
#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                points_.resize(5);
                joint_names_.resize(5);

                for (std::size_t i = 0; i < 5; ++i)
                {
                    ariles_tests::rosmsg::MultiDOFJointTrajectoryPoint point;

                    point.randomize();

                    points_.push_back(point);
                    joint_names_.push_back("xxx");
                }
            }
#endif
        };
    }  // namespace rosmsg
}  // namespace ariles_tests
