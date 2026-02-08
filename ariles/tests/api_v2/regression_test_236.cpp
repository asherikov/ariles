/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include "utility.h"

#ifdef ARILES_VISITOR_namevalue2
#    include <ariles2/visitors/namevalue2.h>
#endif
#include "all_enabled_adapters.h"

#include <ariles2/ariles.h>


// ===============================================================
// TYPES
// ===============================================================

#include "types/complex_auto_declare.h"
#include "types/empty.h"


// ===============================================================
// FIXTURES
// ===============================================================


#include "fixtures/initializers.h"
#include "fixtures/023_write_namevalue2.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER)                                                               \
    ARILES_FIXTURE_TEST_CASE(NameValue2Fixture, VISITOR_ID, NAMESPACE, ConfigurableComplex, INITIALIZER)               \
    ARILES_FIXTURE_TEST_CASE(NameValue2Fixture, VISITOR_ID, NAMESPACE, ConfigurableEmpty, INITIALIZER)

#include "instantiate.h"
