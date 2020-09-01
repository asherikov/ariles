#pragma once


#include <string>
#include <vector>


namespace geometry_msgs
{
    template <class ContainerAllocator>
    struct Vector3_
    {
        typedef Vector3_<ContainerAllocator> Type;

        Vector3_() : x(0.0), y(0.0), z(0.0)
        {
        }
        Vector3_(const ContainerAllocator &_alloc) : x(0.0), y(0.0), z(0.0)
        {
            (void)_alloc;
        }



        typedef double _x_type;
        _x_type x;

        typedef double _y_type;
        _y_type y;

        typedef double _z_type;
        _z_type z;
    };  // struct Vector3_

    typedef ::geometry_msgs::Vector3_<std::allocator<void> > Vector3;
}  // namespace geometry_msgs


namespace ariles_tests
{
    namespace rosmsg
    {
        class Vector3 : public ariles2::DefaultBase, public Eigen::Vector3d
        {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_NAMED_ENTRY(v, x(), x)                                                                                     \
    ARILES2_NAMED_ENTRY(v, y(), y)                                                                                     \
    ARILES2_NAMED_ENTRY(v, z(), z)
#include ARILES2_INITIALIZE

        public:
#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                setRandom();
            }
#endif
        };


        class Vector3Member : public ariles2::DefaultBase
        {
#define ARILES2_ENTRIES(v) ARILES2_TYPED_ENTRY_(v, member, Eigen::Vector3d)
#include ARILES2_INITIALIZE

        public:
#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                member_.setRandom();
            }
#endif
        };
    }  // namespace rosmsg
}  // namespace ariles_tests
