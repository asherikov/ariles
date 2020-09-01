/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include "utility.h"

#ifdef ARILES_VISITOR_graphviz
#    define ARILES2_DEFAULT_VISITORS                                                                                   \
        ARILES2_VISITOR(count)                                                                                         \
        ARILES2_VISITOR(postprocess)                                                                                   \
        ARILES2_VISITOR(preprocess)                                                                                    \
        ARILES2_VISITOR(defaults)                                                                                      \
        ARILES2_VISITOR(read)                                                                                          \
        ARILES2_VISITOR(write)                                                                                         \
        ARILES2_VISITOR(graphviz)

#    include <ariles2/visitors/graphviz.h>
#endif

#include <ariles2/ariles.h>
#include <ariles2/extra.h>

#include "all_enabled_adapters.h"


// ===============================================================
// TYPES
// ===============================================================

#define ARILES_TESTS_COMPARE_DISABLED
#include "types/complex_auto_declare.h"
#include "types/pointers.h"
#undef ARILES_TESTS_COMPARE_DISABLED


// ===============================================================
// FIXTURES
// ===============================================================

#define ARILES_TESTS_TEST_NAME "regression_test_226"
#include "fixtures/initializers.h"
#include "fixtures/016_write_graphviz.h"
#include "fixtures/017_diff.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER)                                                               \
    ARILES_FIXTURE_TEST_CASE(GraphvizFixture, VISITOR_ID, NAMESPACE, ConfigurableComplex, INITIALIZER)                 \
    ARILES_FIXTURE_TEST_CASE(DiffFixture, VISITOR_ID, NAMESPACE, ConfigurableComplex, INITIALIZER)                     \
    ARILES_FIXTURE_TEST_CASE(GraphvizFixture, VISITOR_ID, NAMESPACE, ConfigurablePointers, INITIALIZER)

#include "instantiate.h"
