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

#include "types/special_floats.h"



// ===============================================================
// FIXTURES
// ===============================================================


#include "fixtures/initializers.h"
#include "fixtures/011_check.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER)                                                               \
    ARILES_FIXTURE_TEST_CASE(CheckFixture, VISITOR_ID, NAMESPACE, ConfigurableSpecialFloats, INITIALIZER)


#include "instantiate.h"
