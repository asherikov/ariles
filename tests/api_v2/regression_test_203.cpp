/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"
#include "all_enabled_visitors.h"
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

#include "types/finalize.h"
#include "types/auto_defaults.h"
#define ARILES_TESTS_COMPARE_DISABLED
#include "types/complex_auto_declare.h"
#undef ARILES_TESTS_COMPARE_DISABLED


// ===============================================================
// FIXTURES
// ===============================================================

#include "fixtures/initializers.h"
#include "fixtures/000_basic_interface.h"
#include "fixtures/002_comparison.h"
#include "fixtures/003_comparison_vector.h"
#include "fixtures/006_dummy.h"
#include "fixtures/019_defaults.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER)                                                               \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, VISITOR_ID, NAMESPACE, ConfigurableFinalize, INITIALIZER)          \
    ARILES_FIXTURE_TEST_CASE(ComparisonSimpleFixture, VISITOR_ID, NAMESPACE, ConfigurableFinalize, INITIALIZER)        \
    ARILES_FIXTURE_TEST_CASE(ComparisonMultiFixture, VISITOR_ID, NAMESPACE, ConfigurableFinalize, INITIALIZER)         \
    ARILES_FIXTURE_TEST_CASE(ComparisonVectorFixture, VISITOR_ID, NAMESPACE, ConfigurableFinalize, INITIALIZER)        \
    ARILES_FIXTURE_TEST_CASE(DefaultsCheckFixture, VISITOR_ID, NAMESPACE, ConfigurableFinalize, INITIALIZER)           \
    ARILES_FIXTURE_TEST_CASE(DefaultsCheckFixture, VISITOR_ID, NAMESPACE, ConfigurableComplex, INITIALIZER)


BOOST_FIXTURE_TEST_CASE(Complex_arilesFinalize, ariles_tests::DummyFixture)
{
    ariles_tests::ConfigurableComplex configurable;
    ariles2::apply<ariles2::Finalize>(configurable);
}

BOOST_FIXTURE_TEST_CASE(ConfigurableNoSetDefaults_Defaults, ariles_tests::DummyFixture)
{
    ariles_tests::ConfigurableNoSetDefaults configurable;
    configurable.integer_ = 100;
    ariles2::apply<ariles2::Defaults>(configurable);
    BOOST_CHECK_EQUAL(configurable.integer_, 0);
}


#include "instantiate.h"
