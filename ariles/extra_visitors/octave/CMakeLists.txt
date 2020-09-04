set (TGT_ARILES_VISITOR_LIB      "${PROJECT_NAME}_visitor_${ARILES_VISITOR}")

if(NOT ARILES_DEB_TARGETS)
    add_library(${TGT_ARILES_VISITOR_LIB} SHARED
        ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/writer.cpp)
endif()

include(ariles_install_component)
