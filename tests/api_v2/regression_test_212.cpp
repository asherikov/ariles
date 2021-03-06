/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include "utility.h"

#ifdef ARILES_VISITOR_octave
#    include <ariles2/visitors/octave.h>
#endif
#include "all_enabled_adapters.h"

#include <ariles2/ariles.h>


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


#include "fixtures/initializers.h"
#include "fixtures/010_write_octave.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER)                                                               \
    ARILES_FIXTURE_TEST_CASE(OctaveFixture, VISITOR_ID, NAMESPACE, ConfigurableComplex, INITIALIZER)                   \
    ARILES_FIXTURE_TEST_CASE(OctaveFixture, VISITOR_ID, NAMESPACE, ConfigurablePointers, INITIALIZER)

#include "instantiate.h"
