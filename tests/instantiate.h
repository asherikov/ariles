/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/


#ifdef ARILES_BRIDGE_msgpack
ARILES_TESTS(msgpack, FilenameInitializer)
ARILES_TESTS(msgpack, StreamInitializer)
#endif

#ifdef ARILES_BRIDGE_yaml_cpp03
ARILES_TESTS(yaml_cpp03, FilenameInitializer)
ARILES_TESTS(yaml_cpp03, StreamInitializer)
#endif

#ifdef ARILES_BRIDGE_yaml_cpp
ARILES_TESTS(yaml_cpp, FilenameInitializer)
ARILES_TESTS(yaml_cpp, StreamInitializer)
#endif

#ifdef ARILES_BRIDGE_rapidjson
#define ComparisonMultiFixture ComparisonSimpleFixture
ARILES_TESTS(rapidjson, FilenameInitializer)
ARILES_TESTS(rapidjson, StreamInitializer)
#undef ComparisonMultiFixture
#endif

#ifdef ARILES_BRIDGE_ros
ARILES_TESTS(ros, ROSInitializer)
#endif
