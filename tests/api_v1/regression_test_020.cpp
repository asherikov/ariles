/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"

#include "ariles/adapters_all.h"
#include "ariles/ariles.h"


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


BOOST_FIXTURE_TEST_CASE( CompareInheritance, ariles_tests::DummyFixture )
{
    ariles_tests::ConfigurableDerived configurable1, configurable2;


    ariles::ComparisonParameters param;
    param.double_tolerance_ = g_tolerance;
    param.compare_number_of_entries_ = true;
    param.throw_on_error_ = false;


    configurable1.randomize();
    configurable2 = configurable1;
    BOOST_CHECK(configurable1.arilesCompare(configurable2, param));

    configurable1.randomize();
    BOOST_CHECK(false == configurable1.arilesCompare(configurable2, param));
}
