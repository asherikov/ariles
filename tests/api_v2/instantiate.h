/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#define ARILES_TESTS_SHORTCUT(NAMESPACE, INITIALIZER) ARILES_TESTS(NAMESPACE, NAMESPACE, INITIALIZER)

#ifdef ARILES2_VISITOR_INCLUDED_msgpack
ARILES_TESTS_SHORTCUT(msgpack, FilenameInitializer)
ARILES_TESTS_SHORTCUT(msgpack, StreamInitializer)
#    define ComparisonMultiFixture ComparisonSimpleFixture
ARILES_TESTS_SHORTCUT(msgpack_compact, FilenameInitializer)
ARILES_TESTS_SHORTCUT(msgpack_compact, StreamInitializer)
#    undef ComparisonMultiFixture
#endif

#ifdef ARILES2_VISITOR_INCLUDED_yaml_cpp03
ARILES_TESTS_SHORTCUT(yaml_cpp03, FilenameInitializer)
ARILES_TESTS_SHORTCUT(yaml_cpp03, StreamInitializer)
#endif

#ifdef ARILES2_VISITOR_INCLUDED_yaml_cpp
ARILES_TESTS_SHORTCUT(yaml_cpp, FilenameInitializer)
ARILES_TESTS_SHORTCUT(yaml_cpp, StreamInitializer)
#endif

#ifdef ARILES2_VISITOR_INCLUDED_rapidjson
// A dirty hack to avoid fixture, which is known to fail for JSON.
#    define ComparisonMultiFixture ComparisonSimpleFixture
ARILES_TESTS_SHORTCUT(rapidjson, FilenameInitializer)
ARILES_TESTS_SHORTCUT(rapidjson, StreamInitializer)

#    ifdef ARILES2_VISITOR_INCLUDED_jsonnet
ARILES_TESTS(rapidjson_jsonnet, jsonnet<ariles2::rapidjson>, FilenameInitializer)
ARILES_TESTS(rapidjson_jsonnet, jsonnet<ariles2::rapidjson>, StreamInitializer)
#    endif
#    undef ComparisonMultiFixture
#endif

#ifdef ARILES2_VISITOR_INCLUDED_rosparam
ARILES_TESTS_SHORTCUT(rosparam, ROSInitializer)
#endif

#ifdef ARILES2_VISITOR_INCLUDED_octave
ARILES_TESTS_SHORTCUT(octave, FilenameInitializer)
#endif

#ifdef ARILES2_VISITOR_INCLUDED_namevalue
ARILES_TESTS_SHORTCUT(namevalue, SizeInitializer)
#endif

#ifdef ARILES2_VISITOR_INCLUDED_pugixml
// A dirty hack to avoid fixture, which is known to fail for XML.
#    define ComparisonMultiFixture ComparisonSimpleFixture
ARILES_TESTS_SHORTCUT(pugixml, FilenameInitializer)
ARILES_TESTS_SHORTCUT(pugixml, StreamInitializer)
#    undef ComparisonMultiFixture
#endif

#ifdef ARILES2_VISITOR_INCLUDED_graphviz
ARILES_TESTS_SHORTCUT(graphviz, FilenameInitializer)
#endif
