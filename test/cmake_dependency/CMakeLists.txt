cmake_minimum_required(VERSION 2.8.12)
project(cmakeut_cmake_dependency_test NONE)

set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(cmakeut REQUIRED)

set(CMAKE_MODULE_PATH "${cmakeut_PATH}")

include(FindEigen3)
include(cmakeut_add_external_git_project)
include(cmakeut_copy_dir_if_exists)
include(cmakeut_copy_file_if_exists)
include(cmakeut_detect_func_macro)
include(cmakeut_dump_variables)
include(cmakeut_list_filenames)
