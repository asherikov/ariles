/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include "utility.h"

#ifdef ARILES_BRIDGE_yaml_cpp03
#include "ariles/bridges/yaml_cpp03.h"
#endif

#ifdef ARILES_BRIDGE_yaml_cpp
#include "ariles/bridges/yaml_cpp.h"
#endif

#ifdef ARILES_BRIDGE_msgpack
#include "ariles/bridges/msgpack.h"
#endif

#ifdef ARILES_BRIDGE_ros
#include "ariles/bridges/ros.h"
#endif

#ifdef ARILES_BRIDGE_rapidjson
#include "ariles/bridges/rapidjson.h"
#endif

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


#include "fixtures/initializers.h"
#include "fixtures/000_basic_interface.h"
#include "fixtures/001_constructor_interface.h"
#include "fixtures/002_comparison.h"
#include "fixtures/003_comparison_vector.h"
#include "fixtures/004_comparison_equivalence.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(NAMESPACE, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, NAMESPACE, ConfigurableComplexVerbose, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, NAMESPACE, ConfigurableComplex, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ConstructorInterfaceFixture, NAMESPACE, ConfigurableComplexVerbose, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ConstructorInterfaceFixture, NAMESPACE, ConfigurableComplex, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ComparisonSimpleFixture, NAMESPACE, ConfigurableComplexVerbose, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ComparisonSimpleFixture, NAMESPACE, ConfigurableComplex, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ComparisonMultiFixture, NAMESPACE, ConfigurableComplexVerbose, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ComparisonMultiFixture, NAMESPACE, ConfigurableComplex, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ComparisonVectorFixture, NAMESPACE, ConfigurableComplexVerbose, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ComparisonVectorFixture, NAMESPACE, ConfigurableComplex, INITIALIZER) \
    BOOST_FIXTURE_TEST_CASE(ComparisonEquivalenceFixture##_##NAMESPACE##_##Equivalence##_##INITIALIZER, ComparisonEquivalenceFixture<initializers::INITIALIZER>) \
    { \
        test<ConfigurableComplexVerbose, ConfigurableComplex, ariles::NAMESPACE>(); \
    }


#include "instantiate.h"
