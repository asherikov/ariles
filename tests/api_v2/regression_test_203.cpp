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

#include "ariles/ariles2.h"


// ===============================================================
// TYPES
// ===============================================================

#include "types/postprocess.h"

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


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER)                                                               \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, VISITOR_ID, NAMESPACE, ConfigurablePostProcess, INITIALIZER)       \
    ARILES_FIXTURE_TEST_CASE(ComparisonSimpleFixture, VISITOR_ID, NAMESPACE, ConfigurablePostProcess, INITIALIZER)     \
    ARILES_FIXTURE_TEST_CASE(ComparisonMultiFixture, VISITOR_ID, NAMESPACE, ConfigurablePostProcess, INITIALIZER)      \
    ARILES_FIXTURE_TEST_CASE(ComparisonVectorFixture, VISITOR_ID, NAMESPACE, ConfigurablePostProcess, INITIALIZER)


BOOST_FIXTURE_TEST_CASE(Complex_arilesPostProcess, ariles_tests::DummyFixture)
{
    ariles_tests::ConfigurableComplex configurable;
    ariles::apply<ariles::PostProcess>(configurable);
}


#include "instantiate.h"
