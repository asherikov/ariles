/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#define ARILES_API_VERSION 2

#include "utility.h"

#ifdef ARILES_BRIDGE_octave
#include "ariles/bridges/octave.h"
#endif

#include "ariles/adapters_all.h"
#include "ariles/ariles2.h"


// ===============================================================
// TYPES
// ===============================================================

#include "types_api_v2/complex_auto_declare.h"


// ===============================================================
// FIXTURES
// ===============================================================


#include "fixtures_api_v2/initializers.h"
#include "fixtures_api_v2/010_write_octave.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(BRIDGE_ID, NAMESPACE, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(OctaveFixture, BRIDGE_ID, NAMESPACE, ConfigurableComplex, INITIALIZER)

#include "instantiate.h"