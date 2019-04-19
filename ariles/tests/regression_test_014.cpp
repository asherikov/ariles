/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"

#ifdef ARILES_BRIDGE_yaml_cpp03
#include "ariles/bridges/yaml_cpp03.h"
#endif

#ifdef ARILES_BRIDGE_yaml_cpp
#include "ariles/bridges/yaml_cpp.h"
#endif

#ifdef ARILES_BRIDGE_msgpack
#include "ariles/bridges/msgpack.h"
#endif

#ifdef ARILES_BRIDGE_rapidjson
#include "ariles/bridges/rapidjson.h"
#endif

#ifdef ARILES_BRIDGE_pugixml
#include "ariles/bridges/pugixml.h"
#endif

#include "ariles/adapters_all.h"
#include "ariles/ariles.h"


// ===============================================================
// TYPES
// ===============================================================

#include "types/simple_auto_declare.h"



// ===============================================================
// FIXTURES
// ===============================================================

#include "fixtures/initializers.h"
#include "fixtures/012_basic_interface_add_filename.h"



// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(BRIDGE_ID, NAMESPACE, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, BRIDGE_ID, NAMESPACE, ConfigurableAutoDeclare, INITIALIZER)

#define ARILES_TESTS_SHORTCUT(NAMESPACE, INITIALIZER) ARILES_TESTS(NAMESPACE, NAMESPACE, INITIALIZER)

#ifdef ARILES_BRIDGE_INCLUDED_msgpack
ARILES_TESTS_SHORTCUT(msgpack, FilenameInitializer)
#   define ComparisonMultiFixture ComparisonSimpleFixture
ARILES_TESTS(msgpack_compact, msgpack::compact, FilenameInitializer)
#   undef ComparisonMultiFixture
#endif

#ifdef ARILES_BRIDGE_INCLUDED_yaml_cpp03
ARILES_TESTS_SHORTCUT(yaml_cpp03, FilenameInitializer)
#endif

#ifdef ARILES_BRIDGE_INCLUDED_yaml_cpp
ARILES_TESTS_SHORTCUT(yaml_cpp, FilenameInitializer)
#endif

#ifdef ARILES_BRIDGE_INCLUDED_rapidjson
// A dirty hack to avoid fixture, which is known to fail for JSON.
#   define ComparisonMultiFixture ComparisonSimpleFixture
    ARILES_TESTS_SHORTCUT(rapidjson, FilenameInitializer)
#   ifdef ARILES_BRIDGE_INCLUDED_jsonnet
        ARILES_TESTS(rapidjson_jsonnet, rapidjson::jsonnet, FilenameInitializer)
#   endif
#   undef ComparisonMultiFixture
#endif
