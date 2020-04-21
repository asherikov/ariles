##############################################################################
set(ARILES_TESTS_CONFIG "")

foreach(ARILES_EXTRA_ADAPTER ${ARILES_EXTRA_ADAPTERS})
    string(TOUPPER ${ARILES_EXTRA_ADAPTER} ARILES_EXTRA_ADAPTER_CAPITALIZED)
    set(ARILES_TESTS_CONFIG   "${ARILES_TESTS_CONFIG}#define ARILES_ADAPTER_${ARILES_EXTRA_ADAPTER_CAPITALIZED}\n")
endforeach()


foreach(ARILES_VISITOR ${ARILES_VISITORS})
    if (NOT ${ARILES_VISITOR_${ARILES_VISITOR}} STREQUAL "OFF")
        set(ARILES_TESTS_CONFIG "${ARILES_TESTS_CONFIG}#define ARILES_BRIDGE_${ARILES_VISITOR}\n")
    endif()
endforeach()


file(WRITE "${ARILES_INCLUDE_DIR}/tests/api_v1/tests_config.h" "${ARILES_TESTS_CONFIG}")
include_directories("${ARILES_INCLUDE_DIR}/tests")
##############################################################################


list(APPEND ARILES_TESTING_LIBRARIES  "testlib_api_v1")

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY     "${PROJECT_BINARY_DIR}/tests_bin_api_v1/")


##############################################################################
## API v1
##############################################################################

ariles_define_test(${CMAKE_PROJECT_NAME} "demo_api_v1" "yaml_cpp;ros;octave" "")

# basic test for missing inlines
add_library(testlib_api_v1 SHARED testlib1.cpp testlib2.cpp)


# tests
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "000" "" "${ARILES_TEST_ALL_VISITORS}") # features
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "001" "" "${ARILES_TEST_ALL_VISITORS}") # complex
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "002" "" "${ARILES_TEST_ALL_VISITORS}") # inheritance, strictness
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "003" "" "${ARILES_TEST_ALL_VISITORS}") # finalize
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "004" "" "${ARILES_TEST_ALL_VISITORS}") # no adapters_all.h
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "005" "" "${ARILES_TEST_ALL_VISITORS}") # disabled ariles
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "009" "" "${ARILES_TEST_ALL_VISITORS}") # pointers (scalar)
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "010" "rapidjson;jsonnet" "")           # jsonnet
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "011" "rapidjson;yaml_cpp" "")          # visitor inclusion order #TODO
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "012" "octave" "")                      # octave
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "013" "" "${ARILES_TEST_ALL_VISITORS}") # special floats
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "014" "" "${ARILES_TEST_FILE_VISITORS}")# filenames
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "015" "pugixml" "")                     # XML childs vs attributes
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "016" "" "")                            # inclusion
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "017" "" "${ARILES_TEST_ALL_VISITORS}") # any
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "018" "" "${ARILES_TEST_ALL_VISITORS}") # pointers (configurable)
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "019" "" "")                            # comparison (complex)
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "020" "" "")                            # comparison (inheritance)
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "021" "array" "")                       # array
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "022" "" "${ARILES_TEST_ALL_VISITORS}") # flags
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "023" "rapidjson" "")                   # json floats, flags
