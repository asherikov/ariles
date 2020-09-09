find_package(Boost REQUIRED unit_test_framework timer random system)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM "${CMAKE_CURRENT_LIST_DIR}/common/")


list(APPEND ARILES_TESTING_LIBRARIES  "${Boost_LIBRARIES}")

include(ariles_copy_extra_test_files)
include(ariles_define_regression_test)
include(ariles_define_test)
include(ariles_parse_test_dependencies)

set(ARILES_TEST_FILE_VISITORS "yaml_cpp03;yaml_cpp;msgpack;rapidjson;jsonnet;pugixml")
set(ARILES_TEST_ALL_VISITORS "${ARILES_TEST_FILE_VISITORS};rosparam")

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/api_v2")
