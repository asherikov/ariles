set (TGT_ARILES_BRIDGE_LIB      "ariles_${ARILES_BRIDGE}")
if(NOT ARILES_DEB_TARGETS)
    pkg_search_module(${ARILES_BRIDGE} msgpack)

    if (${ARILES_BRIDGE}_FOUND)
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS "${${ARILES_BRIDGE}_LIBRARY_DIRS}")
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "${${ARILES_BRIDGE}_INCLUDE_DIRS}")
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${${ARILES_BRIDGE}_LIBRARIES}")
    else()
        find_library("ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS" NAMES ${ARILES_BRIDGE})
        if (NOT ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS)
            message(FATAL_ERROR "${ARILES_BRIDGE} is not found.")
        endif()

        find_path("ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES" NAMES msgpack.hpp)
        if (NOT ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES)
            message(FATAL_ERROR "${ARILES_BRIDGE} is not found.")
        endif()
    endif()


    include_directories(${CMAKE_CURRENT_LIST_DIR})

    add_library(${TGT_ARILES_BRIDGE_LIB} SHARED
        ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/reader.cpp
        ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/writer.cpp
        ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/reader_compact.cpp
        ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/writer_compact.cpp)

    add_dependencies(${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE} ${TGT_ARILES_BRIDGE_LIB})
    add_dependencies(${TGT_ARILES_BRIDGE_LIB} TGT_ariles_copy_headers)
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${TGT_ARILES_BRIDGE_LIB};${ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS}")
endif()

include(ariles_install_component)
