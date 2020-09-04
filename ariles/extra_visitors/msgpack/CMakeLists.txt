set (TGT_ARILES_VISITOR_LIB      "${PROJECT_NAME}_visitor_${ARILES_VISITOR}")

if(NOT ARILES_DEB_TARGETS)
    pkg_search_module(${ARILES_VISITOR} msgpack)

    if (${ARILES_VISITOR}_FOUND)
        set(ARILES_VISITOR_${ARILES_VISITOR}_LIBRARY_DIRS "${${ARILES_VISITOR}_LIBRARY_DIRS}")
        set(ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES "${${ARILES_VISITOR}_INCLUDE_DIRS}")
        set(ARILES_VISITOR_${ARILES_VISITOR}_LIBS "${${ARILES_VISITOR}_LIBRARIES}")
    else()
        find_library("ARILES_VISITOR_${ARILES_VISITOR}_LIBS" NAMES ${ARILES_VISITOR})
        if (NOT ARILES_VISITOR_${ARILES_VISITOR}_LIBS)
            message(FATAL_ERROR "${ARILES_VISITOR} is not found.")
        endif()

        find_path("ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES" NAMES msgpack.hpp)
        if (NOT ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES)
            message(FATAL_ERROR "${ARILES_VISITOR} is not found.")
        endif()
    endif()

    link_directories(${ARILES_VISITOR_${ARILES_VISITOR}_LIBRARY_DIRS})
    add_library(${TGT_ARILES_VISITOR_LIB} SHARED
        ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/reader.cpp
        ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/writer.cpp
        ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/reader_compact.cpp
        ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/writer_compact.cpp)
    target_include_directories(${TGT_ARILES_VISITOR_LIB} SYSTEM PRIVATE ${ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES})
endif()

include(ariles_install_component)
