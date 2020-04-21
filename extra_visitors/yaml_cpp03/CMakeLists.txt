list (FIND ARILES_ENABLED_VISITORS "yaml_cpp" INDEX)
if (${INDEX} GREATER -1)
    message(FATAL_ERROR "Conflict between the visitors detected (yaml_cpp -- yaml_cpp03).")
endif()

set (TGT_ARILES_VISITOR_LIB      "ariles_visitor_${ARILES_VISITOR}")
if(NOT ARILES_DEB_TARGETS)
    pkg_search_module(${ARILES_VISITOR} REQUIRED yaml-cpp03)


    set(ARILES_VISITOR_${ARILES_VISITOR}_LIBRARY_DIRS "${${ARILES_VISITOR}_LIBRARY_DIRS}")
    if (EXISTS "${${ARILES_VISITOR}_INCLUDE_DIRS}/yaml-cpp03")
        set(ARILES_VISITOR_${ARILES_VISITOR}_INCLUDE_HEADER "yaml-cpp03/yaml.h")
    else()
        set(ARILES_VISITOR_${ARILES_VISITOR}_INCLUDE_HEADER "yaml-cpp/yaml.h")
    endif()
    set(ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES "${${ARILES_VISITOR}_INCLUDE_DIRS}")
    set(ARILES_VISITOR_${ARILES_VISITOR}_LIBS "${${ARILES_VISITOR}_LIBRARIES}")


    include_directories(${CMAKE_CURRENT_LIST_DIR})
    configure_file("${CMAKE_CURRENT_LIST_DIR}/cmake_in/common.h.in" "${CMAKE_CURRENT_LIST_DIR}/src/common.h")

    link_directories(${ARILES_VISITOR_${ARILES_VISITOR}_LIBRARY_DIRS})
    add_library(${TGT_ARILES_VISITOR_LIB} SHARED
        ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/reader.cpp
        ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/writer.cpp)
endif()

include(ariles_install_component)

