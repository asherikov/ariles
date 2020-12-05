set (TGT_ARILES_VISITOR_LIB      "${PROJECT_NAME}_visitor_${ARILES_VISITOR}")

if(NOT ARILES_DEB_TARGETS)
    find_library("ARILES_VISITOR_${ARILES_VISITOR}_LIBS" "jsonnet")
    if (NOT ARILES_VISITOR_${ARILES_VISITOR}_LIBS)
        message(FATAL_ERROR "Could not find jsonnet libraries. ${ARILES_VISITOR_${ARILES_VISITOR}_LIBS}")
    endif()

    find_path("ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES" "libjsonnet.h")
    if (NOT ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES)
        message(FATAL_ERROR "Could not find jsonnet headers. ${ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES}")
    endif()
endif()

add_library(${TGT_ARILES_VISITOR_LIB} SHARED
    ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/reader.cpp
)

include(ariles_install_component)
