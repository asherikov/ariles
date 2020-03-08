if (ARILES_ADAPTER_EIGEN)
    find_package(Eigen3 REQUIRED)
    include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
endif()
if (ARILES_ADAPTER_BOOST_POINTER)
endif(ARILES_ADAPTER_BOOST_POINTER)


find_package(Boost REQUIRED unit_test_framework timer random system)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})


list(APPEND ARILES_TESTING_LIBRARIES  "${Boost_LIBRARIES};testlib")

include(ariles_copy_extra_test_files)
include(ariles_define_regression_test)
include(ariles_define_test)
include(ariles_parse_test_dependencies)


set (CMAKE_RUNTIME_OUTPUT_DIRECTORY     "${PROJECT_BINARY_DIR}/tests_bin/")

set(ARILES_TEST_FILE_BRIDGES "yaml_cpp03;yaml_cpp;msgpack;rapidjson;jsonnet;pugixml")
set(ARILES_TEST_ALL_BRIDGES "${ARILES_TEST_FILE_BRIDGES};ros")


##############################################################################
## API v1
##############################################################################

ariles_define_test(${CMAKE_PROJECT_NAME} "demo" "yaml_cpp;ros;octave" "")

# basic test for missing inlines
add_library(testlib SHARED testlib1.cpp testlib2.cpp)


# tests
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "000" "" "${ARILES_TEST_ALL_BRIDGES}")  # features
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "001" "" "${ARILES_TEST_ALL_BRIDGES}")  # complex
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "002" "" "${ARILES_TEST_ALL_BRIDGES}")  # inheritance, strictness
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "003" "" "${ARILES_TEST_ALL_BRIDGES}")  # finalize
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "004" "" "${ARILES_TEST_ALL_BRIDGES}")  # no adapters_all.h
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "005" "" "${ARILES_TEST_ALL_BRIDGES}")  # disabled ariles
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "009" "" "${ARILES_TEST_ALL_BRIDGES}")  # pointers (scalar)
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "010" "rapidjson;jsonnet" "")           # jsonnet
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "011" "rapidjson;jsonnet;yaml_cpp" "")          # bridge inclusion order #TODO
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "012" "octave" "")                      # octave
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "013" "" "${ARILES_TEST_ALL_BRIDGES}")  # special floats
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "014" "" "${ARILES_TEST_FILE_BRIDGES}") # filenames
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "015" "pugixml" "")                     # XML childs vs attributes
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "016" "" "")                            # inclusion
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "017" "" "${ARILES_TEST_ALL_BRIDGES}")  # any
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "018" "" "${ARILES_TEST_ALL_BRIDGES}")  # pointers (configurable)
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "019" "" "")                            # comparison (complex)
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "020" "" "")                            # comparison (inheritance)
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "021" "array" "")                       # array
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "022" "" "${ARILES_TEST_ALL_BRIDGES}")  # flags



##############################################################################
## API v2
##############################################################################

ariles_define_test(${CMAKE_PROJECT_NAME} "demo_api_v2" "yaml_cpp;ros;octave" "")

# basic test for missing inlines
add_library(testlib_api_v2 SHARED testlib1_api_v2.cpp testlib2_api_v2.cpp)

# tests
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "200" "" "${ARILES_TEST_ALL_BRIDGES}")  # features
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "201" "" "${ARILES_TEST_ALL_BRIDGES}")  # complex
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "202" "" "${ARILES_TEST_ALL_BRIDGES}")  # inheritance, strictness
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "203" "" "${ARILES_TEST_ALL_BRIDGES}")  # postprocess
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "204" "" "${ARILES_TEST_ALL_BRIDGES}")  # no adapters_all.h
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "205" "" "${ARILES_TEST_ALL_BRIDGES}")  # disabled ariles
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "209" "" "${ARILES_TEST_ALL_BRIDGES}")  # pointers (scalar)
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "210" "rapidjson;jsonnet" "")           # jsonnet
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "211" "rapidjson;jsonnet;yaml_cpp" "")          # bridge inclusion order # TODO
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "212" "octave" "")                      # octave
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "213" "" "${ARILES_TEST_ALL_BRIDGES}")  # special floats
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "214" "" "${ARILES_TEST_FILE_BRIDGES}") # filenames
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "215" "pugixml" "")                     # XML childs vs attributes
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "216" "" "")                            # inclusion
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "217" "" "${ARILES_TEST_ALL_BRIDGES}")  # any
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "218" "" "${ARILES_TEST_ALL_BRIDGES}")  # pointers (configurable)
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "219" "" "")                            # comparison (complex)
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "220" "" "")                            # comparison (inheritance)
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "221" "array" "")                       # array
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "222" "" "${ARILES_TEST_ALL_BRIDGES}")  # flags
