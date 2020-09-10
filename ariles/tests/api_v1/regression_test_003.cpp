/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include "utility.h"
#include "all_enabled_bridges.h"

#include "ariles/adapters_all.h"
#include "ariles/ariles.h"


// ===============================================================
// TYPES
// ===============================================================

#include "types/finalize.h"
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
#include "fixtures/016_set_defaults.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER)                                                               \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, VISITOR_ID, NAMESPACE, ConfigurableFinalize, INITIALIZER)          \
    ARILES_FIXTURE_TEST_CASE(ComparisonSimpleFixture, VISITOR_ID, NAMESPACE, ConfigurableFinalize, INITIALIZER)        \
    ARILES_FIXTURE_TEST_CASE(ComparisonMultiFixture, VISITOR_ID, NAMESPACE, ConfigurableFinalize, INITIALIZER)         \
    ARILES_FIXTURE_TEST_CASE(ComparisonVectorFixture, VISITOR_ID, NAMESPACE, ConfigurableFinalize, INITIALIZER)        \
    ARILES_FIXTURE_TEST_CASE(SetDefaultsCheckFixture, VISITOR_ID, NAMESPACE, ConfigurableFinalize, INITIALIZER)        \
    ARILES_FIXTURE_TEST_CASE(SetDefaultsCheckFixture, VISITOR_ID, NAMESPACE, ConfigurableComplex, INITIALIZER)

BOOST_FIXTURE_TEST_CASE(Complex_arilesFinalize, ariles_tests::DummyFixture)
{
    ariles_tests::ConfigurableComplex configurable;
    configurable.arilesFinalize();
}

BOOST_FIXTURE_TEST_CASE(ConfigurableNoSetDefaults_Defaults, ariles_tests::DummyFixture)
{
    ariles_tests::ConfigurableNoSetDefaults configurable;
    configurable.integer_ = 100;
    configurable.setDefaults();
    BOOST_CHECK_EQUAL(configurable.integer_, 0);
}


#include "instantiate.h"
