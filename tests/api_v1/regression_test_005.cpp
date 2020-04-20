/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"

#define ARILES_DISABLE
#include "ariles/ariles.h"


// ===============================================================
// TYPES
// ===============================================================

#include "types/simple_auto_declare.h"


// ===============================================================
// FIXTURES
// ===============================================================

#include "fixtures/006_dummy.h"


// ===============================================================
// TESTS
// ===============================================================


BOOST_FIXTURE_TEST_CASE( DummyFixture_Configurable, ariles_tests::DummyFixture )
{
    test<ariles_tests::ConfigurableAutoDeclare>();
}
