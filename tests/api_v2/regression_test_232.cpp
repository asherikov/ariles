/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"


#ifdef ARILES_VISITOR_yaml_cpp
#    include <ariles2/visitors/yaml_cpp.h>
#endif

#include "all_enabled_adapters.h"

#include <ariles2/ariles.h>
#include <ariles2/extra.h>


// ===============================================================
// TYPES
// ===============================================================

#include "types/abstract_pointer.h"


// ===============================================================
// FIXTURES
// ===============================================================

#define ARILES_TESTS_TEST_NAME "regression_test_232"
#include "fixtures/initializers.h"
#include "fixtures/017_diff.h"


// ===============================================================
// TESTS
// ===============================================================

namespace ariles_tests
{
    using AbstractNonNullPointer = ariles_tests::AbstractPointer<ariles2::NonNullPointer>;
}

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER)                                                               \
    ARILES_FIXTURE_TEST_CASE(DiffFixture, VISITOR_ID, NAMESPACE, AbstractNonNullPointer, INITIALIZER)

#include "instantiate.h"
