if (ARILES_ADAPTER_EIGEN)
    find_package(Eigen3 REQUIRED)
    include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
endif()
if (ARILES_ADAPTER_BOOST_POINTER)
endif(ARILES_ADAPTER_BOOST_POINTER)


find_package(Boost REQUIRED unit_test_framework timer random system)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

list(APPEND ARILES_MANDATORY_LIBRARIES  "${Boost_LIBRARIES}")

include(ariles_copy_extra_test_files)
include(ariles_define_regression_test)
include(ariles_parse_test_dependencies)


set (CMAKE_RUNTIME_OUTPUT_DIRECTORY     "${PROJECT_BINARY_DIR}/tests_bin/")


# tests
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "000" "" "yaml_cpp03;yaml_cpp;ros;msgpack;rapidjson;jsonnet")
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "001" "" "yaml_cpp03;yaml_cpp;ros;msgpack;rapidjson;jsonnet")
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "002" "" "yaml_cpp03;yaml_cpp;ros;msgpack;rapidjson;jsonnet")
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "003" "" "yaml_cpp03;yaml_cpp;ros;msgpack;rapidjson;jsonnet")
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "004" "" "yaml_cpp03;yaml_cpp;ros;msgpack;rapidjson;jsonnet")
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "005" "" "yaml_cpp03;yaml_cpp;ros;msgpack;rapidjson;jsonnet")
#ariles_define_regression_test(${CMAKE_PROJECT_NAME} "007" "ros")
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "009" "" "yaml_cpp03;yaml_cpp;ros;msgpack;rapidjson;jsonnet")
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "010" "rapidjson;jsonnet" "")
ariles_define_regression_test(${CMAKE_PROJECT_NAME} "011" "rapidjson;yaml_cpp" "")
