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

#include "types_api_v2/simple_flags.h"


// ===============================================================
// FIXTURES
// ===============================================================

#include "fixtures_api_v2/initializers.h"
#include "fixtures_api_v2/014_flags.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(FlagsFixture, VISITOR_ID, NAMESPACE, ConfigurableFlags1 , INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(FlagsFixture, VISITOR_ID, NAMESPACE, ConfigurableFlags2 , INITIALIZER)


#include "instantiate.h"
