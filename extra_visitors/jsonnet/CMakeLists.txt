set (TGT_ARILES_VISITOR_LIB      "ariles_visitor_${ARILES_VISITOR}")

if(NOT ARILES_DEB_TARGETS)
    find_library("ARILES_VISITOR_${ARILES_VISITOR}_LIBS" "jsonnet")
    if (NOT ARILES_VISITOR_${ARILES_VISITOR}_LIBS)
        message(FATAL_ERROR "Could not find jsonnet libraries. ${ARILES_VISITOR_${ARILES_VISITOR}_LIBS}")
    endif()

    find_path("ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES" "libjsonnet.h")
    if (NOT ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES)
        message(FATAL_ERROR "Could not find jsonnet headers. ${ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES}")
    endif()

    link_directories(${ARILES_VISITOR_${ARILES_VISITOR}_LIBRARY_DIRS})

    add_library(${TGT_ARILES_VISITOR_LIB} SHARED
        ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/reader.cpp)
    target_include_directories(${TGT_ARILES_VISITOR_LIB} SYSTEM PRIVATE ${ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES})
endif()

include(ariles_install_component)
