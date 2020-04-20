if (ARILES_ADAPTER_EIGEN)
    find_package(Eigen3 REQUIRED)
    include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
endif()
if (ARILES_ADAPTER_BOOST_POINTER)
endif(ARILES_ADAPTER_BOOST_POINTER)


find_package(Boost REQUIRED unit_test_framework timer random system)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})


list(APPEND ARILES_TESTING_LIBRARIES  "${Boost_LIBRARIES}")

include(ariles_copy_extra_test_files)
include(ariles_define_regression_test)
include(ariles_define_test)
include(ariles_parse_test_dependencies)

set(ARILES_TEST_FILE_VISITORS "yaml_cpp03;yaml_cpp;msgpack;rapidjson;jsonnet;pugixml")
set(ARILES_TEST_ALL_VISITORS "${ARILES_TEST_FILE_VISITORS};ros")

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/api_v1")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/api_v2")
