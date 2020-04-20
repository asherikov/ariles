/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"

#include "ariles/adapters_all.h"
#include "ariles/ariles.h"

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


BOOST_FIXTURE_TEST_CASE( CompareComplex, ariles_tests::DummyFixture )
{
    boost::random::random_device random_generator;
    ariles_tests::ConfigurableComplex configurable1, configurable2;


    ariles::ComparisonParameters param;
    param.double_tolerance_ = g_tolerance;
    param.compare_number_of_entries_ = true;
    param.throw_on_error_ = false;


    configurable1.randomize();
    configurable2 = configurable1;
    BOOST_CHECK(configurable1.arilesCompare(configurable2, param));


    configurable1.randomize();
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.integer_ = GET_RANDOM_INT;
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.unsigned_integer_ = GET_RANDOM_UINT;
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.real_ = GET_RANDOM_REAL;
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.string_ = "blahblah1";
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.std_vector_.pop_back();
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.std_vector_.back() = GET_RANDOM_REAL;
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.std_nested_vector_.pop_back();
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.std_nested_vector_.back().back() = GET_RANDOM_REAL;
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.boolean_false_ = true;
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.enum_ = ariles_tests::SOME_VALUE;
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.better_enum_ = ariles_tests::BetterEnum::DEFINED_1;
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.std_pair_.first = "testtt1";
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.std_pair_.second = GET_RANDOM_REAL;
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    BOOST_CHECK_EQUAL(1, configurable1.std_map_.erase("one1"));
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.std_map_["2two"].push_back("compare_map");
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


#ifdef ARILES_ADAPTER_EIGEN
    configurable1.vector_.setRandom();
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.matrix_.setRandom();
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.matrix_x_.setRandom();
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.std_vector_evector_.back().setRandom();
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.std_nested_vector_evector_.back().back().setRandom();
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.isometry_.matrix() = Eigen::MatrixXd::Random(4,4);
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;


    configurable1.quaternion_.x() = GET_RANDOM_REAL;
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable1 = configurable2;
#endif
}



BOOST_FIXTURE_TEST_CASE( SpecialFloats, ariles_tests::DummyFixture )
{
    ariles_tests::ConfigurableSpecialFloats configurable1, configurable2;


    ariles::ComparisonParameters param;
    param.double_tolerance_ = g_tolerance;
    param.compare_number_of_entries_ = true;
    param.throw_on_error_ = false;


    configurable1.setDefaults();
    configurable2 = configurable1;
    configurable1.float_quiet_nan_ = std::numeric_limits<float>::quiet_NaN();
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable2 = configurable1;
    BOOST_CHECK(configurable1.arilesCompare(configurable2, param));

    configurable1.setDefaults();
    configurable2 = configurable1;
    configurable1.float_signaling_nan_ = std::numeric_limits<float>::signaling_NaN();
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable2 = configurable1;
    BOOST_CHECK(configurable1.arilesCompare(configurable2, param));

    configurable1.setDefaults();
    configurable2 = configurable1;
    configurable1.float_positive_infinity_ = std::numeric_limits<float>::infinity();
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable2.float_positive_infinity_ = -std::numeric_limits<float>::infinity();
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable2 = configurable1;
    BOOST_CHECK(configurable1.arilesCompare(configurable2, param));

    configurable1.setDefaults();
    configurable2 = configurable1;
    configurable1.float_negative_infinity_ = - std::numeric_limits<float>::infinity();
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable2 = configurable1;
    BOOST_CHECK(configurable1.arilesCompare(configurable2, param));


    configurable1.setDefaults();
    configurable2 = configurable1;
    configurable1.double_quiet_nan_ = std::numeric_limits<double>::quiet_NaN();
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable2 = configurable1;
    BOOST_CHECK(configurable1.arilesCompare(configurable2, param));

    configurable1.setDefaults();
    configurable2 = configurable1;
    configurable1.double_signaling_nan_ = std::numeric_limits<double>::signaling_NaN();
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable2 = configurable1;
    BOOST_CHECK(configurable1.arilesCompare(configurable2, param));

    configurable1.setDefaults();
    configurable2 = configurable1;
    configurable1.double_positive_infinity_ = std::numeric_limits<double>::infinity();
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable2.double_positive_infinity_ = -std::numeric_limits<double>::infinity();
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable2 = configurable1;
    BOOST_CHECK(configurable1.arilesCompare(configurable2, param));

    configurable1.setDefaults();
    configurable2 = configurable1;
    configurable1.double_negative_infinity_ = - std::numeric_limits<double>::infinity();
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
    configurable2 = configurable1;
    BOOST_CHECK(configurable1.arilesCompare(configurable2, param));
}
