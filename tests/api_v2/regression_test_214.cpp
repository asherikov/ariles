/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"

#ifdef ARILES_VISITOR_yaml_cpp03
#    include <ariles2/visitors/yaml_cpp03.h>
#endif

#ifdef ARILES_VISITOR_yaml_cpp
#    include <ariles2/visitors/yaml_cpp.h>
#endif

#ifdef ARILES_VISITOR_msgpack
#    include <ariles2/visitors/msgpack.h>
#endif

#ifdef ARILES_VISITOR_jsonnet
#    include <ariles2/visitors/jsonnet.h>
#endif

#ifdef ARILES_VISITOR_rapidjson
#    include <ariles2/visitors/rapidjson.h>
#endif

#ifdef ARILES_VISITOR_pugixml
#    include <ariles2/visitors/pugixml.h>
#endif

#include "all_enabled_adapters.h"

#include <ariles2/ariles.h>


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

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER)                                                               \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, VISITOR_ID, NAMESPACE, ConfigurableAutoDeclare, INITIALIZER)

#define ARILES_TESTS_SHORTCUT(NAMESPACE, INITIALIZER) ARILES_TESTS(NAMESPACE, NAMESPACE, INITIALIZER)

#ifdef ARILES_VISITOR_INCLUDED_msgpack
ARILES_TESTS_SHORTCUT(msgpack, FilenameInitializer)
#    define ComparisonMultiFixture ComparisonSimpleFixture
ARILES_TESTS_SHORTCUT(msgpack_compact, FilenameInitializer)
#    undef ComparisonMultiFixture
#endif

#ifdef ARILES_VISITOR_INCLUDED_yaml_cpp03
ARILES_TESTS_SHORTCUT(yaml_cpp03, FilenameInitializer)
#endif

#ifdef ARILES_VISITOR_INCLUDED_yaml_cpp
ARILES_TESTS_SHORTCUT(yaml_cpp, FilenameInitializer)
#endif

#ifdef ARILES_VISITOR_INCLUDED_rapidjson
// A dirty hack to avoid fixture, which is known to fail for JSON.
#    define ComparisonMultiFixture ComparisonSimpleFixture
ARILES_TESTS_SHORTCUT(rapidjson, FilenameInitializer)
#    undef ComparisonMultiFixture
#endif

#ifdef ARILES_VISITOR_INCLUDED_jsonnet
// A dirty hack to avoid fixture, which is known to fail for JSON.
#    define ComparisonMultiFixture ComparisonSimpleFixture
ARILES_TESTS(rapidjson_jsonnet, jsonnet<ariles::rapidjson>, FilenameInitializer)
#    undef ComparisonMultiFixture
#endif
