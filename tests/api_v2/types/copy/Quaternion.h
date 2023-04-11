#pragma once


#include <string>
#include <vector>


// NOLINTBEGIN(*)
namespace geometry_msgs
{
    template <class ContainerAllocator>
    struct Quaternion_
    {
        typedef Quaternion_<ContainerAllocator> Type;

        Quaternion_() : x(0.0), y(0.0), z(0.0), w(0.0)
        {
        }
        Quaternion_(const ContainerAllocator &_alloc) : x(0.0), y(0.0), z(0.0), w(0.0)
        {
            (void)_alloc;
        }


        typedef double _x_type;
        _x_type x;

        typedef double _y_type;
        _y_type y;

        typedef double _z_type;
        _z_type z;

        typedef double _w_type;
        _w_type w;
    };  // struct Quaternion_

    typedef ::geometry_msgs::Quaternion_<std::allocator<void>> Quaternion;
}  // namespace geometry_msgs
// NOLINTEND(*)


namespace ariles_tests
{
    namespace rosmsg
    {
        class Quaternion : public ariles2::DefaultBase, public Eigen::Quaterniond
        {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_NAMED_ENTRY(v, x(), x)                                                                                     \
    ARILES2_NAMED_ENTRY(v, y(), y)                                                                                     \
    ARILES2_NAMED_ENTRY(v, z(), z)                                                                                     \
    ARILES2_NAMED_ENTRY(v, w(), w)
#include ARILES2_INITIALIZE

        public:
#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                setIdentity();
            }
#endif
        };


        class QuaternionMember : public ariles2::DefaultBase
        {
#define ARILES2_ENTRIES(v) ARILES2_TYPED_ENTRY_(v, member, Eigen::Quaterniond)
#include ARILES2_INITIALIZE

        public:
#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                member_.setIdentity();
            }
#endif
        };
    }  // namespace rosmsg
}  // namespace ariles_tests
