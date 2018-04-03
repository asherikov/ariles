/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include "utility.h"

#ifdef ARILES_BRIDGE_yaml_cpp03
#include "ariles/formats/yaml_cpp03.h"
#endif

#ifdef ARILES_BRIDGE_yaml_cpp
#include "ariles/formats/yaml_cpp.h"
#endif

#include "ariles/formats/msgpack.h"
#include "ariles/adapters_all.h"
#include "ariles/ariles.h"


// ===============================================================
// TYPES
// ===============================================================

#include "types/complex.h"
#include "types/complex_verbose.h"



// ===============================================================
// FIXTURES
// ===============================================================


#include "fixtures/base_default.h"
#include "fixtures/000_basic_interface.h"
#include "fixtures/001_constructor_interface.h"
#include "fixtures/002_comparison.h"
#include "fixtures/003_comparison_vector.h"
#include "fixtures/004_comparison_equivalence.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(NAMESPACE) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, NAMESPACE, ConfigurableComplexVerbose) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, NAMESPACE, ConfigurableComplex) \
    ARILES_FIXTURE_TEST_CASE(ConstructorInterfaceFixture, NAMESPACE, ConfigurableComplexVerbose) \
    ARILES_FIXTURE_TEST_CASE(ConstructorInterfaceFixture, NAMESPACE, ConfigurableComplex) \
    ARILES_FIXTURE_TEST_CASE(ComparisonSimpleFixture, NAMESPACE, ConfigurableComplexVerbose) \
    ARILES_FIXTURE_TEST_CASE(ComparisonSimpleFixture, NAMESPACE, ConfigurableComplex) \
    ARILES_FIXTURE_TEST_CASE(ComparisonMultiFixture, NAMESPACE, ConfigurableComplexVerbose) \
    ARILES_FIXTURE_TEST_CASE(ComparisonMultiFixture, NAMESPACE, ConfigurableComplex) \
    ARILES_FIXTURE_TEST_CASE(ComparisonVectorFixture, NAMESPACE, ConfigurableComplexVerbose) \
    ARILES_FIXTURE_TEST_CASE(ComparisonVectorFixture, NAMESPACE, ConfigurableComplex) \
    BOOST_FIXTURE_TEST_CASE( ComparisonEquivalenceFixture##_##NAMESPACE##_##Equivalence, ComparisonEquivalenceFixture) \
    { \
        test<ConfigurableComplexVerbose, ConfigurableComplex, ariles::NAMESPACE>(); \
    }

ARILES_TESTS(msgpack)
#ifdef ARILES_BRIDGE_yaml_cpp03
ARILES_TESTS(yaml_cpp03)
#endif
#ifdef ARILES_BRIDGE_yaml_cpp
ARILES_TESTS(yaml_cpp)
#endif
