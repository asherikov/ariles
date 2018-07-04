/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#define ARILES_TESTS_SHORTCUT(NAMESPACE, INITIALIZER) ARILES_TESTS(NAMESPACE, NAMESPACE, INITIALIZER)

#ifdef ARILES_BRIDGE_msgpack
ARILES_TESTS_SHORTCUT(msgpack, FilenameInitializer)
ARILES_TESTS_SHORTCUT(msgpack, StreamInitializer)
#endif

#ifdef ARILES_BRIDGE_yaml_cpp03
ARILES_TESTS_SHORTCUT(yaml_cpp03, FilenameInitializer)
ARILES_TESTS_SHORTCUT(yaml_cpp03, StreamInitializer)
#endif

#ifdef ARILES_BRIDGE_yaml_cpp
ARILES_TESTS_SHORTCUT(yaml_cpp, FilenameInitializer)
ARILES_TESTS_SHORTCUT(yaml_cpp, StreamInitializer)
#endif

#ifdef ARILES_BRIDGE_rapidjson
// A dirty hack to avoid fixture, which is known to fail for JSON.
#   define ComparisonMultiFixture ComparisonSimpleFixture
    ARILES_TESTS_SHORTCUT(rapidjson, FilenameInitializer)
    ARILES_TESTS_SHORTCUT(rapidjson, StreamInitializer)
#   ifdef ARILES_BRIDGE_jsonnet
        ARILES_TESTS(rapidjson_jsonnet, rapidjson::jsonnet, FilenameInitializer)
        ARILES_TESTS(rapidjson_jsonnet, rapidjson::jsonnet, StreamInitializer)
#   endif
#   undef ComparisonMultiFixture
#endif

#ifdef ARILES_BRIDGE_ros
ARILES_TESTS_SHORTCUT(ros, ROSInitializer)
#endif
