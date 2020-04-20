list(APPEND ARILES_TESTING_LIBRARIES  "testlib_api_v2")

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY     "${PROJECT_BINARY_DIR}/tests_bin_api_v2/")


##############################################################################
## API v2
##############################################################################

ariles_define_test(${CMAKE_PROJECT_NAME} "demo_api_v2" "yaml_cpp;ros;octave" "")

# basic test for missing inlines
add_library(testlib_api_v2 SHARED testlib1.cpp testlib2.cpp)


# tests
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "200" "" "${ARILES_TEST_ALL_VISITORS}") # features
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "201" "" "${ARILES_TEST_ALL_VISITORS}") # complex
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "202" "" "${ARILES_TEST_ALL_VISITORS}") # inheritance, strictness
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "203" "" "${ARILES_TEST_ALL_VISITORS}") # postprocess
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "204" "" "${ARILES_TEST_ALL_VISITORS}") # no adapters_all.h
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
