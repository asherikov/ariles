/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"
#include "all_enabled_adapters.h"

#define ARILES2_DEFAULT_VISITORS                                                                                       \
    ARILES2_VISITOR(count)                                                                                             \
    ARILES2_VISITOR(count_missing)                                                                                     \
    ARILES2_VISITOR(finalize)                                                                                          \
    ARILES2_VISITOR(prewrite)                                                                                          \
    ARILES2_VISITOR(defaults)                                                                                          \
    ARILES2_VISITOR(read)                                                                                              \
    ARILES2_VISITOR(write)                                                                                             \
    ARILES2_VISITOR(compare)

#include <ariles2/visitors/compare.h>
#include <ariles2/ariles.h>

// ===============================================================
// TYPES
// ===============================================================

#include "types/complex_auto_declare.h"
#include "types/special_floats.h"


// ===============================================================
// FIXTURES
// ===============================================================

#include "fixtures/006_dummy.h"


// ===============================================================
// TESTS
// ===============================================================


BOOST_FIXTURE_TEST_CASE(CompareComplex, ariles_tests::DummyFixture)
{
    boost::random::random_device random_generator;
    ariles_tests::ConfigurableComplex configurable1;
    ariles_tests::ConfigurableComplex configurable2;


    ariles2::Compare visitor;
    ariles2::Compare::Parameters param;
    param.double_tolerance_ = g_tolerance;
    param.compare_number_of_entries_ = true;


    configurable1.randomize();
    configurable2 = configurable1;
    compareAndCheckWithBacktrace(visitor, configurable1, configurable2, param, "Complex comparison 1 failed");


    configurable1.randomize();
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Complex comparison 2 unexpectedly passed");
    configurable1 = configurable2;


    configurable1.integer_ = GET_RANDOM_INT;
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Integer comparison unexpectedly passed");
    configurable1 = configurable2;


    configurable1.unsigned_integer_ = GET_RANDOM_UINT;
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Unsigned integer comparison unexpectedly passed");
    configurable1 = configurable2;


    configurable1.real_ = GET_RANDOM_REAL;
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Real comparison unexpectedly passed");
    configurable1 = configurable2;


    configurable1.string_ = configurable2.string_ + "x";
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "String comparison unexpectedly passed");
    configurable1 = configurable2;


    configurable1.std_vector_.pop_back();
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Std vector pop comparison unexpectedly passed");
    configurable1 = configurable2;


    configurable1.std_vector_.back() = GET_RANDOM_REAL;
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Std vector back comparison unexpectedly passed");
    configurable1 = configurable2;


    configurable1.std_nested_vector_.pop_back();
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Std nested vector pop comparison unexpectedly passed");
    configurable1 = configurable2;


    configurable1.std_nested_vector_.back().back() = GET_RANDOM_REAL;
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Std nested vector back comparison unexpectedly passed");
    configurable1 = configurable2;


    configurable1.boolean_false_ = true;
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Boolean false comparison unexpectedly passed");
    configurable1 = configurable2;


    configurable1.some_enum_ = ariles_tests::SOME_VALUE;
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Some enum comparison unexpectedly passed");
    configurable1 = configurable2;


    configurable1.better_enum_ = ariles_tests::BetterEnum::DEFINED_1;
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Better enum comparison unexpectedly passed");
    configurable1 = configurable2;


    configurable1.std_pair_.first = configurable2.std_pair_.first + "x";
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Std pair first comparison unexpectedly passed");
    configurable1 = configurable2;


    configurable1.std_pair_.second = GET_RANDOM_REAL;
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Std pair second comparison unexpectedly passed");
    configurable1 = configurable2;


    BOOST_CHECK_EQUAL(1, configurable1.std_map_.erase("one1"));
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Std map erase comparison unexpectedly passed");
    configurable1 = configurable2;


    configurable1.std_map_["2two"].emplace_back("compare_map");
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Std map emplace comparison unexpectedly passed");
    configurable1 = configurable2;


#ifdef ARILES_ADAPTER_EIGEN
    configurable1.vector_.setRandom();
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Vector setRandom comparison unexpectedly passed");
    configurable1 = configurable2;


    configurable1.matrix_.setRandom();
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Matrix setRandom comparison unexpectedly passed");
    configurable1 = configurable2;


    configurable1.matrix_x_.setRandom();
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Matrix X setRandom comparison unexpectedly passed");
    configurable1 = configurable2;


    configurable1.std_vector_evector_.back().setRandom();
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Std vector evector comparison unexpectedly passed");
    configurable1 = configurable2;


    configurable1.std_nested_vector_evector_.back().back().setRandom();
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Std nested vector evector comparison unexpectedly passed");
    configurable1 = configurable2;


    configurable1.isometry_.matrix() = Eigen::MatrixXd::Random(4, 4);
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Isometry matrix comparison unexpectedly passed");
    configurable1 = configurable2;


    configurable1.quaternion_.x() = GET_RANDOM_REAL;
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Quaternion X comparison unexpectedly passed");
    configurable1 = configurable2;
#endif
}



BOOST_FIXTURE_TEST_CASE(SpecialFloats, ariles_tests::DummyFixture)
{
    ariles_tests::ConfigurableSpecialFloats configurable1;
    ariles_tests::ConfigurableSpecialFloats configurable2;


    ariles2::Compare visitor;
    ariles2::Compare::Parameters param;
    param.double_tolerance_ = g_tolerance;
    param.compare_number_of_entries_ = true;


    ariles2::apply<ariles2::Defaults>(configurable1);
    configurable2 = configurable1;
    configurable1.float_quiet_nan_ = std::numeric_limits<float>::quiet_NaN();
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Float quiet NaN comparison unexpectedly passed");
    configurable2 = configurable1;
    compareAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Float quiet NaN equality comparison failed");

    ariles2::apply<ariles2::Defaults>(configurable1);
    configurable2 = configurable1;
    configurable1.float_signaling_nan_ = std::numeric_limits<float>::signaling_NaN();
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Float signaling NaN comparison unexpectedly passed");
    configurable2 = configurable1;
    compareAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Float signaling NaN equality comparison failed");

    ariles2::apply<ariles2::Defaults>(configurable1);
    configurable2 = configurable1;
    configurable1.float_positive_infinity_ = std::numeric_limits<float>::infinity();
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Float positive infinity comparison unexpectedly passed");
    configurable2.float_positive_infinity_ = -std::numeric_limits<float>::infinity();
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Float pos/neg infinity comparison unexpectedly passed");
    configurable2 = configurable1;
    compareAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Float positive infinity equality comparison failed");

    ariles2::apply<ariles2::Defaults>(configurable1);
    configurable2 = configurable1;
    configurable1.float_negative_infinity_ = -std::numeric_limits<float>::infinity();
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Float negative infinity comparison unexpectedly passed");
    configurable2 = configurable1;
    compareAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Float negative infinity equality comparison failed");


    ariles2::apply<ariles2::Defaults>(configurable1);
    configurable2 = configurable1;
    configurable1.double_quiet_nan_ = std::numeric_limits<double>::quiet_NaN();
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Double quiet NaN comparison unexpectedly passed");
    configurable2 = configurable1;
    compareAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Double quiet NaN equality comparison failed");

    ariles2::apply<ariles2::Defaults>(configurable1);
    configurable2 = configurable1;
    configurable1.double_signaling_nan_ = std::numeric_limits<double>::signaling_NaN();
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Double signaling NaN comparison unexpectedly passed");
    configurable2 = configurable1;
    compareAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Double signaling NaN equality comparison failed");

    ariles2::apply<ariles2::Defaults>(configurable1);
    configurable2 = configurable1;
    configurable1.double_positive_infinity_ = std::numeric_limits<double>::infinity();
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Double positive infinity comparison unexpectedly passed");
    configurable2.double_positive_infinity_ = -std::numeric_limits<double>::infinity();
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Double pos/neg infinity comparison unexpectedly passed");
    configurable2 = configurable1;
    compareAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Double positive infinity equality comparison failed");

    ariles2::apply<ariles2::Defaults>(configurable1);
    configurable2 = configurable1;
    configurable1.double_negative_infinity_ = -std::numeric_limits<double>::infinity();
    compareExpectingFailureAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Double negative infinity comparison unexpectedly passed");
    configurable2 = configurable1;
    compareAndCheckWithBacktrace(
            visitor, configurable1, configurable2, param, "Double negative infinity equality comparison failed");
}
