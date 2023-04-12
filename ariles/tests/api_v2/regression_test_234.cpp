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
#include <ariles2/extra.h>


// ===============================================================
// TYPES
// ===============================================================

#include "types/abstract_pointer.h"


// ===============================================================
// FIXTURES
// ===============================================================

#define ARILES_TESTS_TEST_NAME "regression_test_234"
#include "fixtures/initializers.h"
#include "fixtures/021_optional_comparison.h"


// ===============================================================
// TESTS
// ===============================================================

namespace ariles_tests
{
    using AbstractOptionalPointer = ariles_tests::AbstractPointer<ariles2::OptionalPointer, static_cast<int>(false)>;
}

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER)                                                               \
    ARILES_FIXTURE_TEST_CASE(ComparisonSimpleFixture, VISITOR_ID, NAMESPACE, AbstractOptionalPointer, INITIALIZER)

// wont work
#define ARILES2_TESTS_DISABLE_msgpack_compact
#include "instantiate.h"
