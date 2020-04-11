include_directories(${CMAKE_CURRENT_LIST_DIR})

set (TGT_ARILES_BRIDGE_LIB      "ariles_visitor_${ARILES_BRIDGE}")

if(NOT ARILES_DEB_TARGETS)
    include_directories(${CMAKE_CURRENT_LIST_DIR})

    add_library(${TGT_ARILES_BRIDGE_LIB} SHARED
        ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/writer.cpp)

    add_dependencies(${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE} ${TGT_ARILES_BRIDGE_LIB})
    add_dependencies(${TGT_ARILES_BRIDGE_LIB} TGT_ariles_copy_headers)

    set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${TGT_ARILES_BRIDGE_LIB}")
endif()

include(ariles_install_component)
