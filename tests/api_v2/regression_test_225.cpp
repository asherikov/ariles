/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include "utility.h"

#undef ARILES_TEST_DEFAULT_BASE
#define ARILES_TEST_DEFAULT_BASE ariles2::RelaxedSloppyBase


#include "all_enabled_visitors.h"
#include "all_enabled_adapters.h"

#define ARILES2_DEFAULT_VISITORS                                                                                       \
    ARILES2_VISITOR(count)                                                                                             \
    ARILES2_VISITOR(postread)                                                                                       \
    ARILES2_VISITOR(preprocess)                                                                                        \
    ARILES2_VISITOR(defaults)                                                                                          \
    ARILES2_VISITOR(read)                                                                                              \
    ARILES2_VISITOR(write)                                                                                             \
    ARILES2_VISITOR(compare)

#include <ariles2/visitors/compare.h>
#include <ariles2/ariles.h>
#include <ariles2/extra.h>


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
#include "fixtures/002_comparison.h"
#include "fixtures/003_comparison_vector.h"
#include "fixtures/004_comparison_equivalence.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER)                                                               \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, VISITOR_ID, NAMESPACE, ConfigurableComplexVerbose, INITIALIZER)    \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, VISITOR_ID, NAMESPACE, ConfigurableComplex, INITIALIZER)           \
    ARILES_FIXTURE_TEST_CASE(ComparisonSimpleFixture, VISITOR_ID, NAMESPACE, ConfigurableComplexVerbose, INITIALIZER)  \
    ARILES_FIXTURE_TEST_CASE(ComparisonSimpleFixture, VISITOR_ID, NAMESPACE, ConfigurableComplex, INITIALIZER)         \
    ARILES_FIXTURE_TEST_CASE(ComparisonMultiFixture, VISITOR_ID, NAMESPACE, ConfigurableComplexVerbose, INITIALIZER)   \
    ARILES_FIXTURE_TEST_CASE(ComparisonMultiFixture, VISITOR_ID, NAMESPACE, ConfigurableComplex, INITIALIZER)          \
    ARILES_FIXTURE_TEST_CASE(ComparisonVectorFixture, VISITOR_ID, NAMESPACE, ConfigurableComplexVerbose, INITIALIZER)  \
    ARILES_FIXTURE_TEST_CASE(ComparisonVectorFixture, VISITOR_ID, NAMESPACE, ConfigurableComplex, INITIALIZER)         \
    BOOST_FIXTURE_TEST_CASE(                                                                                           \
            ComparisonEquivalenceFixture##_##VISITOR_ID##_##Equivalence##_##INITIALIZER,                               \
            ariles_tests::ComparisonEquivalenceFixture<ariles_tests::initializers::INITIALIZER>)                       \
    {                                                                                                                  \
        test<ariles_tests::ConfigurableComplexVerbose, ariles_tests::ConfigurableComplex, ariles2::NAMESPACE>();       \
    }


#include "instantiate.h"
