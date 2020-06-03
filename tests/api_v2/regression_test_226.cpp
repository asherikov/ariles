/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include "utility.h"

#include <ariles2/ariles.h>

#ifdef ARILES_VISITOR_graphviz
#    include <ariles2/visitors/graphviz.h>
#endif
#include "all_enabled_adapters.h"


// ===============================================================
// TYPES
// ===============================================================

#include "types/complex_auto_declare.h"


// ===============================================================
// FIXTURES
// ===============================================================


#include "fixtures/initializers.h"
#include "fixtures/016_write_graphviz.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER)                                                               \
    ARILES_FIXTURE_TEST_CASE(GraphvizFixture, VISITOR_ID, NAMESPACE, ConfigurableComplex, INITIALIZER)

#include "instantiate.h"
