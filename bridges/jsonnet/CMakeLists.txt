set (TGT_ARILES_BRIDGE_LIB      "ariles_visitor_${ARILES_BRIDGE}")

if(NOT ARILES_DEB_TARGETS)
    include_directories(${CMAKE_CURRENT_LIST_DIR})

    find_library("ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS" "jsonnet")
    if (NOT ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS)
        message(FATAL_ERROR "Could not find jsonnet libraries. ${ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS}")
    endif()

    find_path("ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES" "libjsonnet.h")
    if (NOT ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES)
        message(FATAL_ERROR "Could not find jsonnet headers. ${ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES}")
    endif()

    link_directories(${ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS})

    add_library(${TGT_ARILES_BRIDGE_LIB} SHARED
        ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/reader.cpp)
endif()

include(ariles_install_component)
