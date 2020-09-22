/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"
#include "all_enabled_adapters.h"

#define ARILES2_DEFAULT_VISITORS                                                                                       \
    ARILES2_VISITOR(count)                                                                                             \
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

#include "types/inheritance.h"


// ===============================================================
// FIXTURES
// ===============================================================

#include "fixtures/006_dummy.h"


// ===============================================================
// TESTS
// ===============================================================


BOOST_FIXTURE_TEST_CASE(CompareInheritance, ariles_tests::DummyFixture)
{
    ariles_tests::ConfigurableDerived configurable1, configurable2;


    ariles2::Compare visitor;
    ariles2::Compare::Parameters param;
    param.double_tolerance_ = g_tolerance;
    param.compare_number_of_entries_ = true;


    configurable1.randomize();
    configurable2 = configurable1;
    BOOST_CHECK(true == ariles2::apply(visitor, configurable1, configurable2, param));

    configurable1.randomize();
    BOOST_CHECK(false == ariles2::apply(visitor, configurable1, configurable2, param));
}
