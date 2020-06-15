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


file(WRITE "${ARILES_INCLUDE_DIR}/tests/api_v2/tests_config.h" "${ARILES_TESTS_CONFIG}")
include_directories("${ARILES_INCLUDE_DIR}/tests")
##############################################################################


list(APPEND ARILES_TESTING_LIBRARIES  "testlib_api_v2")

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY     "${PROJECT_BINARY_DIR}/tests_bin_api_v2/")


##############################################################################
## API v2
##############################################################################

ariles_define_test(${CMAKE_PROJECT_NAME} "demo_api_v2" "yaml_cpp;ros;octave" "")

# basic test for missing inlines
add_library(testlib_api_v2 SHARED testlib1.cpp testlib2.cpp)
target_include_directories(testlib_api_v2 PRIVATE ${ARILES_CORE_BUILD_INCLUDES})
target_include_directories(testlib_api_v2 SYSTEM PRIVATE ${ARILES_CORE_DEPENDENCY_INCLUDES})


# tests
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "200" "" "${ARILES_TEST_ALL_VISITORS}") # features
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "201" "" "${ARILES_TEST_ALL_VISITORS}") # complex
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "202" "" "${ARILES_TEST_ALL_VISITORS}") # inheritance, strictness
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "203" "" "${ARILES_TEST_ALL_VISITORS}") # postprocess
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "204" "" "${ARILES_TEST_ALL_VISITORS}") # no all_enabled_adapters.h
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "205" "" "${ARILES_TEST_ALL_VISITORS}") # disabled ariles
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "209" "" "${ARILES_TEST_ALL_VISITORS}") # pointers (scalar)
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "210" "rapidjson;jsonnet" "")           # jsonnet
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "211" "rapidjson;yaml_cpp" "")          # visitor inclusion order # TODO
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "212" "octave" "")                      # octave
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "213" "" "${ARILES_TEST_ALL_VISITORS}") # special floats
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "214" "" "${ARILES_TEST_FILE_VISITORS}")# filenames
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "215" "pugixml" "")                     # XML childs vs attributes
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "216" "" "")                            # inclusion
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "217" "" "${ARILES_TEST_ALL_VISITORS}") # any
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "218" "" "${ARILES_TEST_ALL_VISITORS}") # pointers (configurable)
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "219" "" "")                            # comparison (complex)
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "220" "" "")                            # comparison (inheritance)
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "221" "array" "")                       # array
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "222" "" "${ARILES_TEST_ALL_VISITORS}") # flags
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "223" "rapidjson" "")                   # json floats, flags
if (NOT ${ARILES_VISITOR_yaml_cpp} STREQUAL "OFF")
    ariles_define_regression_test(${CMAKE_PROJECT_NAME} "224" "yaml_cpp" "")                # missing entries
elseif (NOT ${ARILES_VISITOR_yaml_cpp03} STREQUAL "OFF")
    ariles_define_regression_test(${CMAKE_PROJECT_NAME} "224" "yaml_cpp03" "")              # missing entries
endif()
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "225" "" "${ARILES_TEST_ALL_VISITORS}") # 201, but with sloppy pairs and maps
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "226" "graphviz" "")                    # complex
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "227" "graphviz" "")                    # pointers
