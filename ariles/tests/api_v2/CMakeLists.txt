##############################################################################
set(ARILES_TESTS_CONFIG "")

foreach(ARILES_EXTRA_ADAPTER ${ARILES_EXTRA_ADAPTERS})
    string(TOUPPER ${ARILES_EXTRA_ADAPTER} ARILES_EXTRA_ADAPTER_CAPITALIZED)
    set(ARILES_TESTS_CONFIG   "${ARILES_TESTS_CONFIG}#define ARILES_ADAPTER_${ARILES_EXTRA_ADAPTER_CAPITALIZED}\n")
endforeach()


foreach(ARILES_VISITOR ${ARILES_VISITORS})
    if (NOT ${ARILES_VISITOR_${ARILES_VISITOR}} STREQUAL "OFF")
        set(ARILES_TESTS_CONFIG "${ARILES_TESTS_CONFIG}#define ARILES_VISITOR_${ARILES_VISITOR}\n")
    endif()
endforeach(ARILES_VISITOR)


file(WRITE "${PROJECT_BINARY_DIR}/tests/api_v2/tests_config.h" "${ARILES_TESTS_CONFIG}")
include_directories("${PROJECT_BINARY_DIR}/tests")
##############################################################################


list(APPEND ARILES_TESTING_LIBRARIES  "testlib_api_v2")

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY     "${PROJECT_BINARY_DIR}/tests_bin_api_v2/")


##############################################################################
## API v2
##############################################################################

ariles_define_test(${PROJECT_NAME} "demo_api_v2" "yaml_cpp;rosparam;octave" "")

# basic test for missing inlines
add_library(testlib_api_v2 SHARED testlib1.cpp testlib2.cpp)
target_include_directories(testlib_api_v2 PRIVATE ${ARILES_CORE_BUILD_INCLUDES})
target_include_directories(testlib_api_v2 SYSTEM PRIVATE ${ARILES_CORE_DEPENDENCY_INCLUDES})


# tests
ariles_define_regression_test("200" "ANY"   "${ARILES_TEST_ALL_VISITORS}")  # features
ariles_define_regression_test("201" "ANY"   "${ARILES_TEST_ALL_VISITORS}")  # complex
ariles_define_regression_test("202" "ANY"   "${ARILES_TEST_ALL_VISITORS}")  # inheritance, strictness
ariles_define_regression_test("203" ""      "${ARILES_TEST_ALL_VISITORS}")  # finalize, defaults
ariles_define_regression_test("204" "ANY"   "${ARILES_TEST_ALL_VISITORS}")  # no all_enabled_adapters.h
ariles_define_regression_test("205" ""      "${ARILES_TEST_ALL_VISITORS}")  # disabled ariles
ariles_define_regression_test("209" "ANY"   "${ARILES_TEST_ALL_VISITORS}")  # pointers (scalar)
ariles_define_regression_test("210" "rapidjson;jsonnet")                    # jsonnet
ariles_define_regression_test("211" "rapidjson;yaml_cpp")                   # visitor inclusion order # TODO
ariles_define_regression_test("212" "octave")                               # octave: complex, pointers
ariles_define_regression_test("213" "ANY"   "${ARILES_TEST_ALL_VISITORS}")  # special floats
ariles_define_regression_test("214" "ANY"   "${ARILES_TEST_FILE_VISITORS}") # filenames
ariles_define_regression_test("215" "pugixml")                              # XML childs vs attributes
ariles_define_regression_test("216" "")                                     # inclusion
ariles_define_regression_test("217" "ANY"   "${ARILES_TEST_ALL_VISITORS}")  # any
ariles_define_regression_test("218" "ANY"   "${ARILES_TEST_ALL_VISITORS}")  # pointers (configurable)
ariles_define_regression_test("219" "")                                     # comparison (complex)
ariles_define_regression_test("220" "")                                     # comparison (inheritance)
ariles_define_regression_test("221" "namevalue")                            # namevalue
ariles_define_regression_test("222" "ANY"   "${ARILES_TEST_ALL_VISITORS}")  # flags
ariles_define_regression_test("223" "rapidjson")                            # json floats, flags
ariles_define_regression_test("224" "yaml_cpp")                             # missing entries
ariles_define_regression_test("225" "ANY"   "${ARILES_TEST_ALL_VISITORS}")  # 201, but with sloppy pairs and maps
ariles_define_regression_test("226" "graphviz")                             # graphviz: complex, pointers
ariles_define_regression_test("229" "graphviz")                             # graphviz: ariles_diagram
ariles_define_regression_test("230" "")                                     # copyto, copyfrom
set(CMAKE_CXX_CLANG_TIDY_BACK "${CMAKE_CXX_CLANG_TIDY}")
set(CMAKE_CXX_CLANG_TIDY "")
ariles_define_regression_test("231" "protobuf3")                            # protobuf
if (${ARILES_VISITOR_protobuf3} STREQUAL "ON")
    protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${CMAKE_CURRENT_LIST_DIR}/types/protobuf/test.proto)
    add_library("protobuf_test" STATIC ${PROTO_SRCS} ${PROTO_HDRS})
    set_property(TARGET "protobuf_test" APPEND_STRING PROPERTY COMPILE_FLAGS " -Wno-error=shadow -Wno-error=unused-parameter ")
    set_property(TARGET ${TGT_TEST} APPEND_STRING PROPERTY COMPILE_FLAGS " -Wno-error=shadow -Wno-error=unused-parameter ")
    set_property(TARGET ${TGT_TEST} APPEND PROPERTY INCLUDE_DIRECTORIES ${CMAKE_CURRENT_BINARY_DIR})
    target_link_libraries(${TGT_TEST} protobuf_test)
endif()
set(CMAKE_CXX_CLANG_TIDY "${CMAKE_CXX_CLANG_TIDY_BACK}")
ariles_define_regression_test("232" "yaml_cpp")                             # NonNullPointer abstract
ariles_define_regression_test("233" "ANY"   "${ARILES_TEST_ALL_VISITORS}")  # subtree read
