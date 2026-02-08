set (TGT_ARILES_VISITOR_LIB      "${PROJECT_NAME}_visitor_${ARILES_VISITOR}")

add_library(${TGT_ARILES_VISITOR_LIB}
    ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/writer.cpp
)

set(ARILES_COMPONENT_CMAKE_DEPS "")
include(ariles_install_component)
