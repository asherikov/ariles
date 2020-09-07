/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#define ARILES_API_VERSION 2

#include "utility.h"
#include "all_enabled_visitors.h"

#include "ariles/adapters_all.h"
#include "ariles/ariles2.h"


// ===============================================================
// TYPES
// ===============================================================

#include "types/postprocess.h"
#include "types/no_setdefaults.h"

#define ARILES_TESTS_BOOST_UTF_DISABLED
#include "types/complex_auto_declare.h"
#undef ARILES_TESTS_BOOST_UTF_DISABLED


// ===============================================================
// FIXTURES
// ===============================================================

#include "fixtures/initializers.h"
#include "fixtures/000_basic_interface.h"
#include "fixtures/002_comparison.h"
#include "fixtures/003_comparison_vector.h"
#include "fixtures/006_dummy.h"
#include "fixtures/016_defaults.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER)                                                               \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, VISITOR_ID, NAMESPACE, ConfigurablePostProcess, INITIALIZER)       \
    ARILES_FIXTURE_TEST_CASE(ComparisonSimpleFixture, VISITOR_ID, NAMESPACE, ConfigurablePostProcess, INITIALIZER)     \
    ARILES_FIXTURE_TEST_CASE(ComparisonMultiFixture, VISITOR_ID, NAMESPACE, ConfigurablePostProcess, INITIALIZER)      \
    ARILES_FIXTURE_TEST_CASE(ComparisonVectorFixture, VISITOR_ID, NAMESPACE, ConfigurablePostProcess, INITIALIZER)     \
    ARILES_FIXTURE_TEST_CASE(DefaultsCheckFixture, VISITOR_ID, NAMESPACE, ConfigurablePostProcess, INITIALIZER)        \
    ARILES_FIXTURE_TEST_CASE(DefaultsCheckFixture, VISITOR_ID, NAMESPACE, ConfigurableComplex, INITIALIZER)


BOOST_FIXTURE_TEST_CASE(Complex_arilesPostProcess, ariles_tests::DummyFixture)
{
    ariles_tests::ConfigurableComplex configurable;
    ariles::apply<ariles::PostProcess>(configurable);
}

BOOST_FIXTURE_TEST_CASE(ConfigurableNoSetDefaults_Defaults, ariles_tests::DummyFixture)
{
    ariles_tests::ConfigurableNoSetDefaults configurable;
    configurable.integer_ = 100;
    ariles::apply<ariles::Defaults>(configurable);
    BOOST_CHECK_EQUAL(configurable.integer_, 0);
}

#include "instantiate.h"
