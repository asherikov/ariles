/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"


#include <ariles2/visitors/rapidjson.h>
#include <ariles2/ariles.h>
#include <ariles2/visitors/yaml_cpp.h>


// ===============================================================
// TYPES
// ===============================================================

#include "types/simple_auto_declare.h"


// ===============================================================
// FIXTURES
// ===============================================================

#include "fixtures/initializers.h"
#include "fixtures/000_basic_interface.h"



// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER)                                                               \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, VISITOR_ID, NAMESPACE, ConfigurableAutoDeclare, INITIALIZER)

#include "instantiate.h"
