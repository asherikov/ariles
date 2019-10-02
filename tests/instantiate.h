/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#define ARILES_TESTS_SHORTCUT(NAMESPACE, INITIALIZER) ARILES_TESTS(NAMESPACE, NAMESPACE, INITIALIZER)

#ifdef ARILES_BRIDGE_INCLUDED_msgpack
ARILES_TESTS_SHORTCUT(msgpack, FilenameInitializer)
ARILES_TESTS_SHORTCUT(msgpack, StreamInitializer)
#   define ComparisonMultiFixture ComparisonSimpleFixture
ARILES_TESTS(msgpack_compact, msgpack::compact, FilenameInitializer)
ARILES_TESTS(msgpack_compact, msgpack::compact, StreamInitializer)
#   undef ComparisonMultiFixture
#endif

#ifdef ARILES_BRIDGE_INCLUDED_yaml_cpp03
ARILES_TESTS_SHORTCUT(yaml_cpp03, FilenameInitializer)
ARILES_TESTS_SHORTCUT(yaml_cpp03, StreamInitializer)
#endif

#ifdef ARILES_BRIDGE_INCLUDED_yaml_cpp
ARILES_TESTS_SHORTCUT(yaml_cpp, FilenameInitializer)
ARILES_TESTS_SHORTCUT(yaml_cpp, StreamInitializer)
#endif

#ifdef ARILES_BRIDGE_INCLUDED_rapidjson
// A dirty hack to avoid fixture, which is known to fail for JSON.
#   define ComparisonMultiFixture ComparisonSimpleFixture
    ARILES_TESTS_SHORTCUT(rapidjson, FilenameInitializer)
    ARILES_TESTS_SHORTCUT(rapidjson, StreamInitializer)
#   ifdef ARILES_BRIDGE_INCLUDED_jsonnet
        ARILES_TESTS(rapidjson_jsonnet, rapidjson::jsonnet, FilenameInitializer)
        ARILES_TESTS(rapidjson_jsonnet, rapidjson::jsonnet, StreamInitializer)
#   endif
#   undef ComparisonMultiFixture
#endif

#ifdef ARILES_BRIDGE_INCLUDED_ros
ARILES_TESTS_SHORTCUT(ros, ROSInitializer)
#endif

#ifdef ARILES_BRIDGE_INCLUDED_octave
ARILES_TESTS_SHORTCUT(octave, FilenameInitializer)
#endif

#ifdef ARILES_BRIDGE_INCLUDED_array
ARILES_TESTS_SHORTCUT(array, SizeInitializer)
#endif

#ifdef ARILES_BRIDGE_INCLUDED_pugixml
// A dirty hack to avoid fixture, which is known to fail for XML.
#   define ComparisonMultiFixture ComparisonSimpleFixture
    ARILES_TESTS_SHORTCUT(pugixml, FilenameInitializer)
    ARILES_TESTS_SHORTCUT(pugixml, StreamInitializer)
#   undef ComparisonMultiFixture
#endif
