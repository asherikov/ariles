/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include "utility.h"

#include "all_enabled_visitors.h"
#include "all_enabled_adapters.h"

#define ARILES2_DEFAULT_VISITORS                                                                                       \
    ARILES2_VISITOR(count)                                                                                             \
    ARILES2_VISITOR(count_missing)                                                                                     \
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

#include "types/plain.h"


// ===============================================================
// FIXTURES
// ===============================================================

#include "fixtures/initializers.h"
#include "fixtures/026_plain_visitors.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER)                                                               \
    BOOST_FIXTURE_TEST_CASE(                                                                                           \
            PlainVisitorFixture_##VISITOR_ID##_##INITIALIZER,                                                          \
            ariles_tests::PlainVisitorFixture<ariles_tests::initializers::INITIALIZER>)                                \
    {                                                                                                                  \
        test<ariles_tests::ConfigurablePlain, ariles2::NAMESPACE>();                                                   \
    }

#include "instantiate.h"
