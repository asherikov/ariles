list (FIND ARILES_ENABLED_BRIDGES "yaml_cpp" INDEX)
if (${INDEX} GREATER -1)
    message(FATAL_ERROR "Conflict between the bridges detected (yaml_cpp -- yaml_cpp03).")
endif()

set (TGT_ARILES_BRIDGE_LIB      "ariles_visitor_${ARILES_BRIDGE}")
if(NOT ARILES_DEB_TARGETS)
    pkg_search_module(${ARILES_BRIDGE} REQUIRED yaml-cpp03)


    set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS "${${ARILES_BRIDGE}_LIBRARY_DIRS}")
    if (EXISTS "${${ARILES_BRIDGE}_INCLUDE_DIRS}/yaml-cpp03")
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDE_HEADER "yaml-cpp03/yaml.h")
    else()
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDE_HEADER "yaml-cpp/yaml.h")
    endif()
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "${${ARILES_BRIDGE}_INCLUDE_DIRS}")
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${TGT_ARILES_BRIDGE_LIB};${${ARILES_BRIDGE}_LIBRARIES}")


    include_directories(${CMAKE_CURRENT_LIST_DIR})
    configure_file("${CMAKE_CURRENT_LIST_DIR}/cmake_in/common.h.in" "${CMAKE_CURRENT_LIST_DIR}/src/common.h")

    link_directories(${ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS})
    add_library(${TGT_ARILES_BRIDGE_LIB} SHARED
        ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/reader.cpp
        ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/writer.cpp)

    add_dependencies(${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE} ${TGT_ARILES_BRIDGE_LIB})
    add_dependencies(${TGT_ARILES_BRIDGE_LIB} TGT_ariles_copy_headers)
endif()

include(ariles_install_component)

