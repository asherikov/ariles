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

#include "types/simple_verbose.h"
#include "types/simple_auto_declare.h"
#include "types/no_setdefaults.h"
#include "types/empty.h"



// ===============================================================
// FIXTURES
// ===============================================================

#include "fixtures/initializers.h"
#include "fixtures/000_basic_interface.h"



// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER)                                                               \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, VISITOR_ID, NAMESPACE, ConfigurableVerbose, INITIALIZER)           \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, VISITOR_ID, NAMESPACE, ConfigurableEmpty, INITIALIZER)             \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, VISITOR_ID, NAMESPACE, ConfigurableAutoDeclare, INITIALIZER)       \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, VISITOR_ID, NAMESPACE, ConfigurableNoSetDefaults, INITIALIZER)

#include "instantiate.h"
