/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2016 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <api_v2/tests_config.h>

#include <iostream>

#define BOOST_TEST_MODULE ariles
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN

#include <boost/test/unit_test.hpp>
#include <boost/test/results_reporter.hpp>
#include <boost/timer/timer.hpp>
#include <boost/lexical_cast.hpp>

#include <boost/random/random_device.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>

#include <ariles2/visitors/compare.h>


#define ARILES_TEST_DEFAULT_BASE ariles2::DefaultBase

namespace ariles_tests
{
    struct GlobalFixtureConfig
    {
        GlobalFixtureConfig()
        {
            // boost::unit_test::unit_test_log.set_threshold_level(
            // boost::unit_test::log_successful_tests );
            boost::unit_test::results_reporter::set_level(boost::unit_test::DETAILED_REPORT);
        }
        ~GlobalFixtureConfig()
        {
        }
    };
}  // namespace ariles_tests


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
// Depending on Boost version a compiler may issue a warning about extra ';',
// at the same time, compilation may fail on some systems if ';' is omitted.
namespace ariles_tests
{
    BOOST_GLOBAL_FIXTURE(GlobalFixtureConfig);
}
#pragma GCC diagnostic pop


#define ARILES_FIXTURE_TEST_CASE(FIXTURE_NAME, VISITOR_ID, FORMAT_NAMESPACE, CONFIGURABLE_TYPE, INITIALIZER_TYPE)      \
    BOOST_FIXTURE_TEST_CASE(                                                                                           \
            FIXTURE_NAME##_##VISITOR_ID##_##CONFIGURABLE_TYPE##_##INITIALIZER_TYPE,                                    \
            ariles_tests::FIXTURE_NAME<ariles_tests::initializers::INITIALIZER_TYPE>)                                  \
    {                                                                                                                  \
        test<ariles_tests::CONFIGURABLE_TYPE, ariles2::FORMAT_NAMESPACE>();                                            \
    }

#define ARILES_FIXTURE_TEST_CASE_2CLASSES(                                                                             \
        FIXTURE_NAME, VISITOR_ID, FORMAT_NAMESPACE, CONFIGURABLE_TYPE1, CONFIGURABLE_TYPE2, INITIALIZER_TYPE)          \
    BOOST_FIXTURE_TEST_CASE(                                                                                           \
            FIXTURE_NAME##_##VISITOR_ID##_##CONFIGURABLE_TYPE1##_##CONFIGURABLE_TYPE2##_##INITIALIZER_TYPE,            \
            ariles_tests::FIXTURE_NAME<ariles_tests::initializers::INITIALIZER_TYPE>)                                  \
    {                                                                                                                  \
        test<ariles_tests::CONFIGURABLE_TYPE1, ariles_tests::CONFIGURABLE_TYPE2, ariles2::FORMAT_NAMESPACE>();         \
    }

// -----
// random

// crashes on destruction in FreeBSD
// boost::random::random_device                g_random_generator;

boost::random::uniform_int_distribution<int> g_int_uniform_distribution(
        std::numeric_limits<int>::min(),
        std::numeric_limits<int>::max());

boost::random::uniform_int_distribution<unsigned int> g_uint_uniform_distribution(
        std::numeric_limits<unsigned int>::min(),
        std::numeric_limits<int>::max());

boost::random::uniform_real_distribution<> g_real_uniform_distribution(-1e5, 1e5);

#define GET_RANDOM_UINT g_uint_uniform_distribution(random_generator)
#define GET_RANDOM_INT g_int_uniform_distribution(random_generator)
#define GET_RANDOM_REAL g_real_uniform_distribution(random_generator)
// -----


const double g_tolerance = 1e-12;


namespace ariles_tests
{
    /**
     * @brief Print the backtrace from the Compare visitor
     * @param visitor The Compare visitor whose backtrace to print
     * @param message A descriptive message about the comparison
     */
    inline void printComparisonBacktrace(const ariles2::Compare &visitor, const std::string &message)
    {
        std::cout << message << " Backtrace: ";
        for (const std::string &trace : visitor.backtrace_)
        {
            std::cout << "[" << trace << "] -> ";
        }
        std::cout << std::endl;
    }

    /**
     * @brief Perform a comparison and print backtrace if it fails
     * @param visitor The Compare visitor to use
     * @param left Left operand for comparison
     * @param right Right operand for comparison
     * @param param Parameters for comparison
     * @param failure_message Message to print if comparison fails
     * @return True if comparison passes, false otherwise
     */
    template <typename T_LEFT, typename T_RIGHT>
    bool compareWithBacktrace(
            ariles2::Compare &visitor,
            const T_LEFT &left,
            const T_RIGHT &right,
            const ariles2::Compare::Parameters &param,
            const std::string &failure_message)
    {
        bool comparison_result = ariles2::apply(visitor, left, right, param);
        if (!comparison_result)
        {
            printComparisonBacktrace(visitor, failure_message);
        }
        return comparison_result;
    }

    /**
     * @brief Perform a comparison expecting failure and print backtrace if it unexpectedly passes
     * @param visitor The Compare visitor to use
     * @param left Left operand for comparison
     * @param right Right operand for comparison
     * @param param Parameters for comparison
     * @param success_message Message to print if comparison unexpectedly passes
     * @return True if comparison fails as expected, false otherwise
     */
    template <typename T_LEFT, typename T_RIGHT>
    bool compareExpectingFailureWithBacktrace(
            ariles2::Compare &visitor,
            const T_LEFT &left,
            const T_RIGHT &right,
            const ariles2::Compare::Parameters &param,
            const std::string &success_message)
    {
        bool comparison_result = ariles2::apply(visitor, left, right, param);
        if (comparison_result)
        {
            printComparisonBacktrace(visitor, success_message);
        }
        return comparison_result;
    }

    /**
     * @brief Perform a comparison expecting success and print backtrace if it fails
     * @param visitor The Compare visitor to use
     * @param left Left operand for comparison
     * @param right Right operand for comparison
     * @param param Parameters for comparison
     * @param failure_message Message to print if comparison fails
     * @return True if comparison passes as expected, false otherwise
     */
    template <typename T_LEFT, typename T_RIGHT>
    bool compareExpectingSuccessWithBacktrace(
            ariles2::Compare &visitor,
            const T_LEFT &left,
            const T_RIGHT &right,
            const ariles2::Compare::Parameters &param,
            const std::string &failure_message)
    {
        return compareWithBacktrace(visitor, left, right, param, failure_message);
    }

    /**
     * @brief Perform a comparison with BOOST_CHECK and print backtrace if it fails
     * @param visitor The Compare visitor to use
     * @param left Left operand for comparison
     * @param right Right operand for comparison
     * @param param Parameters for comparison
     * @param failure_message Message to print if comparison fails
     */
    template <typename T_LEFT, typename T_RIGHT>
    void compareAndCheckWithBacktrace(
            ariles2::Compare &visitor,
            const T_LEFT &left,
            const T_RIGHT &right,
            const ariles2::Compare::Parameters &param,
            const std::string &failure_message)
    {
        bool comparison_result = ariles2::apply(visitor, left, right, param);
        if (!comparison_result)
        {
            printComparisonBacktrace(visitor, failure_message);
        }
        BOOST_CHECK(comparison_result);
    }

    /**
     * @brief Perform a comparison expecting failure with BOOST_CHECK and print backtrace if it unexpectedly passes
     * @param visitor The Compare visitor to use
     * @param left Left operand for comparison
     * @param right Right operand for comparison
     * @param param Parameters for comparison
     * @param success_message Message to print if comparison unexpectedly passes
     */
    template <typename T_LEFT, typename T_RIGHT>
    void compareExpectingFailureAndCheckWithBacktrace(
            ariles2::Compare &visitor,
            const T_LEFT &left,
            const T_RIGHT &right,
            const ariles2::Compare::Parameters &param,
            const std::string &success_message)
    {
        bool comparison_result = ariles2::apply(visitor, left, right, param);
        if (comparison_result)
        {
            printComparisonBacktrace(visitor, success_message);
        }
        BOOST_CHECK(!comparison_result);
    }
}  // namespace ariles_tests
