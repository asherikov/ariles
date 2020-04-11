include_directories(${CMAKE_CURRENT_LIST_DIR})

set (TGT_ARILES_BRIDGE_LIB      "ariles_visitor_${ARILES_BRIDGE}")

if(NOT ARILES_DEB_TARGETS)
    include_directories(${CMAKE_CURRENT_LIST_DIR})

    add_library(${TGT_ARILES_BRIDGE_LIB} SHARED
        ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/writer.cpp)
endif()

include(ariles_install_component)
