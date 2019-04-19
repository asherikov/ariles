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

#include "types/complex_auto_declare.h"
#include "types/complex_verbose.h"



// ===============================================================
// FIXTURES
// ===============================================================


#include "fixtures/initializers.h"
#include "fixtures/000_basic_interface.h"
#include "fixtures/001_constructor_interface.h"
#include "fixtures/002_comparison.h"
#include "fixtures/003_comparison_vector.h"
#include "fixtures/004_comparison_equivalence.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(BRIDGE_ID, NAMESPACE, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, BRIDGE_ID, NAMESPACE, ConfigurableComplexVerbose, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, BRIDGE_ID, NAMESPACE, ConfigurableComplex, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ConstructorInterfaceFixture, BRIDGE_ID, NAMESPACE, ConfigurableComplexVerbose, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ConstructorInterfaceFixture, BRIDGE_ID, NAMESPACE, ConfigurableComplex, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ComparisonSimpleFixture, BRIDGE_ID, NAMESPACE, ConfigurableComplexVerbose, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ComparisonSimpleFixture, BRIDGE_ID, NAMESPACE, ConfigurableComplex, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ComparisonMultiFixture, BRIDGE_ID, NAMESPACE, ConfigurableComplexVerbose, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ComparisonMultiFixture, BRIDGE_ID, NAMESPACE, ConfigurableComplex, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ComparisonVectorFixture, BRIDGE_ID, NAMESPACE, ConfigurableComplexVerbose, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ComparisonVectorFixture, BRIDGE_ID, NAMESPACE, ConfigurableComplex, INITIALIZER) \
    BOOST_FIXTURE_TEST_CASE(ComparisonEquivalenceFixture##_##BRIDGE_ID##_##Equivalence##_##INITIALIZER, ComparisonEquivalenceFixture<initializers::INITIALIZER>) \
    { \
        test<ConfigurableComplexVerbose, ConfigurableComplex, ariles::NAMESPACE>(); \
    }


#include "instantiate.h"
