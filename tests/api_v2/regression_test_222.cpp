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

#include <ariles2/ariles.h>


// ===============================================================
// TYPES
// ===============================================================

#include "types/simple_flags.h"


// ===============================================================
// FIXTURES
// ===============================================================

#include "fixtures/initializers.h"
#include "fixtures/014_flags.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER)                                                               \
    ARILES_FIXTURE_TEST_CASE(FlagsFixture, VISITOR_ID, NAMESPACE, ConfigurableFlags1, INITIALIZER)                     \
    ARILES_FIXTURE_TEST_CASE(FlagsFixture, VISITOR_ID, NAMESPACE, ConfigurableFlags2, INITIALIZER)


#include "instantiate.h"
