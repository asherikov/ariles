/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define BOOST_TEST_MODULE ariles
#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include <boost/test/results_reporter.hpp>
#include <boost/timer/timer.hpp>
#include <boost/lexical_cast.hpp>

#include <boost/random/random_device.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>



struct GlobalFixtureConfig
{
    GlobalFixtureConfig()
    {
        //boost::unit_test::unit_test_log.set_threshold_level( boost::unit_test::log_successful_tests );
        boost::unit_test::results_reporter::set_level( boost::unit_test::DETAILED_REPORT );
    }
    ~GlobalFixtureConfig() {}
};


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
// Depending on Boost version a compiler may issue a warning about extra ';',
// at the same time, compilation may fail on some systems if ';' is omitted.
BOOST_GLOBAL_FIXTURE( GlobalFixtureConfig ) ;
#pragma GCC diagnostic pop


#define ARILES_FIXTURE_TEST_CASE(FIXTURE_NAME, BRIDGE_ID, FORMAT_NAMESPACE, CONFIGURABLE_TYPE, INITIALIZER_TYPE) \
    BOOST_FIXTURE_TEST_CASE( FIXTURE_NAME##_##BRIDGE_ID##_##CONFIGURABLE_TYPE##_##INITIALIZER_TYPE, FIXTURE_NAME<initializers::INITIALIZER_TYPE> ) \
    { \
        test<CONFIGURABLE_TYPE, ariles::FORMAT_NAMESPACE>(); \
    }

#define ARILES_FIXTURE_TEST_CASE_2CLASSES(FIXTURE_NAME, BRIDGE_ID, FORMAT_NAMESPACE, CONFIGURABLE_TYPE1, CONFIGURABLE_TYPE2, INITIALIZER_TYPE) \
    BOOST_FIXTURE_TEST_CASE( FIXTURE_NAME##_##BRIDGE_ID##_##CONFIGURABLE_TYPE1##_##CONFIGURABLE_TYPE2##_##INITIALIZER_TYPE, FIXTURE_NAME<initializers::INITIALIZER_TYPE> ) \
    { \
        test<CONFIGURABLE_TYPE1, CONFIGURABLE_TYPE2, ariles::FORMAT_NAMESPACE>(); \
    }

// -----
// random
boost::random::random_device                g_random_generator;

boost::random::uniform_int_distribution<int>   g_int_uniform_distribution(
        std::numeric_limits<int>::min(),
        std::numeric_limits<int>::max());

boost::random::uniform_int_distribution<unsigned int>   g_uint_uniform_distribution(
        std::numeric_limits<unsigned int>::min(),
        std::numeric_limits<int>::max());

boost::random::uniform_real_distribution<>  g_real_uniform_distribution(-1e5, 1e5);

#define GET_RANDOM_UINT     g_uint_uniform_distribution(g_random_generator);
#define GET_RANDOM_INT      g_int_uniform_distribution(g_random_generator);
#define GET_RANDOM_REAL     g_real_uniform_distribution(g_random_generator);
// -----


const double g_tolerance = 1e-12;

#include <tests_config.h>
