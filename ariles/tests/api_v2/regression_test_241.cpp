/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"

#ifdef ARILES_VISITOR_ros2param
#    include <ariles2/visitors/ros2param.h>
#endif

#include "all_enabled_adapters.h"

#define ARILES2_DEFAULT_VISITORS                                                                                       \
    ARILES2_VISITOR(count)                                                                                             \
    ARILES2_VISITOR(count_missing)                                                                                     \
    ARILES2_VISITOR(finalize)                                                                                          \
    ARILES2_VISITOR(prewrite)                                                                                          \
    ARILES2_VISITOR(defaults)                                                                                          \
    ARILES2_VISITOR(read)                                                                                              \
    ARILES2_VISITOR(write)

#include <ariles2/ariles.h>


// ===============================================================
// FIXTURES
// ===============================================================

#include "fixtures/006_dummy.h"


// ===============================================================
// TESTS
// ===============================================================

namespace
{
    struct UpdateParamsTest : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, double_member, double)                                                                     \
    ARILES2_TYPED_ENTRY_(v, bool_member, bool)

#include ARILES2_INITIALIZE

        virtual ~UpdateParamsTest() = default;
    };
}  // namespace


BOOST_FIXTURE_TEST_CASE(MissingEntries, ariles_tests::DummyFixture)
{
    rclcpp::init(/*argn=*/0, /*argv=*/nullptr);

    auto options =
            rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(
                    true);

    UpdateParamsTest params;

    // Allow updating partially specified params from another namespace
    ariles2::ns_ros2param::Reader::Parameters parameters;
    parameters.allow_missing_entries_ = true;

    params.bool_member_ = true;

    const rclcpp::Node::SharedPtr sub_2(new rclcpp::Node("update_param_test_3", options));
    sub_2->declare_parameter("bool_member", false);
    ariles2::apply<ariles2::ns_ros2param::Reader>(sub_2->get_node_parameters_interface(), params, parameters);
    BOOST_CHECK_EQUAL(params.bool_member_, false);  // THIS ONE FAILS, bool_member is true
}
