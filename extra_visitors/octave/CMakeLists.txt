include_directories(${CMAKE_CURRENT_LIST_DIR})

set (TGT_ARILES_VISITOR_LIB      "ariles_visitor_${ARILES_VISITOR}")

if(NOT ARILES_DEB_TARGETS)
    include_directories(${CMAKE_CURRENT_LIST_DIR})

    add_library(${TGT_ARILES_VISITOR_LIB} SHARED
        ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/writer.cpp)
endif()

include(ariles_install_component)
