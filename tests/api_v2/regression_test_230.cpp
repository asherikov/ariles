/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include "utility.h"

#ifdef ARILES_ADAPTER_EIGEN
#    ifdef ARILES_ADAPTER_ROSMSG
#        if __cplusplus >= 201103L
#            define ENABLE_ROSMSG
#        endif
#    endif
#endif



#define ARILES2_DEFAULT_VISITORS                                                                                   \
    ARILES2_VISITOR(count)                                                                                         \
    ARILES2_VISITOR(postprocess)                                                                                   \
    ARILES2_VISITOR(preprocess)                                                                                    \
    ARILES2_VISITOR(defaults)                                                                                      \
    ARILES2_VISITOR(read)                                                                                          \
    ARILES2_VISITOR(write)                                                                                         \
    ARILES2_VISITOR(copyto)                                                                                        \
    ARILES2_VISITOR(copyfrom)                                                                                      \
    ARILES2_VISITOR(compare)

#include <ariles2/visitors/copyto.h>
#include <ariles2/visitors/copyfrom.h>
#include <ariles2/visitors/compare.h>


#include <ariles2/ariles.h>
#include <ariles2/extra.h>

#include "all_enabled_adapters.h"

#ifdef ENABLE_ROSMSG
#    include "types/copy/Header.h"
#    include "types/copy/Vector3.h"
#    include "types/copy/Quaternion.h"
#    include "types/copy/Twist.h"
#    include "types/copy/Transform.h"
#    include "types/copy/MultiDOFJointTrajectory.h"
#    include "types/copy/MultiDOFJointTrajectoryPoint.h"

#    include <ariles2/adapters/rosmsg_geometry_msgs.h>
#endif


// ===============================================================
// TYPES
// ===============================================================

#define ARILES_TESTS_COMPARE_DISABLED
#include "types/complex_auto_declare.h"
#include "types/copy/complex.h"
#include "types/pointers.h"
#include "types/copy/pointers.h"

#undef ARILES_TESTS_COMPARE_DISABLED


// ===============================================================
// FIXTURES
// ===============================================================

#include "fixtures/018_copy_compare.h"


#define ARILES_FIXTURE_TEST_CASE_COPY_COMPARE(ID, CONFIGURABLE_TYPE1, CONFIGURABLE_TYPE2)                              \
    BOOST_FIXTURE_TEST_CASE(CopyCompareFixture_##ID, ariles_tests::CopyCompareFixture)                                 \
    {                                                                                                                  \
        test<ariles2::CopyTo, ariles2::CopyFrom, ariles_tests::CONFIGURABLE_TYPE1, CONFIGURABLE_TYPE2>();              \
    }


// ===============================================================
// TESTS
// ===============================================================

ARILES_FIXTURE_TEST_CASE_COPY_COMPARE(Complex, ConfigurableComplex, ariles_tests::CopyComplex)
ARILES_FIXTURE_TEST_CASE_COPY_COMPARE(Pointers, ConfigurablePointers, ariles_tests::CopyPointers)

#ifdef ENABLE_ROSMSG
namespace ariles_tests
{
    template <class t_Member>
    class Container
    {
    public:
        t_Member member;
    };
}  // namespace ariles_tests


ARILES_FIXTURE_TEST_CASE_COPY_COMPARE(rosmsgVector3, rosmsg::Vector3, geometry_msgs::Vector3);
ARILES_FIXTURE_TEST_CASE_COPY_COMPARE(
        rosmsgVector3Member,
        rosmsg::Vector3Member,
        ariles_tests::Container<geometry_msgs::Vector3>);

ARILES_FIXTURE_TEST_CASE_COPY_COMPARE(rosmsgQuaternion, rosmsg::Quaternion, geometry_msgs::Quaternion);
ARILES_FIXTURE_TEST_CASE_COPY_COMPARE(
        rosmsgQuaternionMember,
        rosmsg::QuaternionMember,
        ariles_tests::Container<geometry_msgs::Quaternion>);

ARILES_FIXTURE_TEST_CASE_COPY_COMPARE(rosmsgTransform, rosmsg::Transform, geometry_msgs::Transform);
ARILES_FIXTURE_TEST_CASE_COPY_COMPARE(
        rosmsgTransformMember,
        rosmsg::TransformMember,
        ariles_tests::Container<geometry_msgs::Transform>);

ARILES_FIXTURE_TEST_CASE_COPY_COMPARE(rosmsgTwist, rosmsg::Twist, geometry_msgs::Twist);

ARILES_FIXTURE_TEST_CASE_COPY_COMPARE(
        rosmsgMultiDOFJointTrajectoryPoint,
        rosmsg::MultiDOFJointTrajectoryPoint,
        trajectory_msgs::MultiDOFJointTrajectoryPoint);

ARILES_FIXTURE_TEST_CASE_COPY_COMPARE(
        rosmsgMultiDOFJointTrajectory,
        rosmsg::MultiDOFJointTrajectory,
        trajectory_msgs::MultiDOFJointTrajectory);
#endif
