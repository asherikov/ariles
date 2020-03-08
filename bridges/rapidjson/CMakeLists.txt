list (FIND ARILES_ENABLED_BRIDGES "jsonnet" INDEX)
if (${INDEX} GREATER -1)
    set(${ARILES_BRIDGE}_WITH_JSONNET ON)
endif()


set (TGT_ARILES_BRIDGE_LIB      "ariles_${ARILES_BRIDGE}")

if(NOT ARILES_DEB_TARGETS)
    pkg_search_module(${ARILES_BRIDGE} RapidJSON)


    if (${ARILES_BRIDGE}_FOUND)
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS "${${ARILES_BRIDGE}_LIBRARY_DIRS}")
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "${${ARILES_BRIDGE}_INCLUDE_DIRS}")
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${${ARILES_BRIDGE}_LIBRARIES}")
    else()
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS "")
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "")

        find_path("ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES" NAMES rapidjson/rapidjson.hpp)
        if (NOT ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES)
            message(FATAL_ERROR "${ARILES_BRIDGE} is not found.")
        endif()
    endif()


    include_directories(${CMAKE_CURRENT_LIST_DIR})

    if (${ARILES_BRIDGE}_WITH_JSONNET)
        add_library(${TGT_ARILES_BRIDGE_LIB} SHARED
            ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/jsonnet_reader.cpp
            ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/reader.cpp
            ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/writer.cpp)
    else()
        add_library(${TGT_ARILES_BRIDGE_LIB} SHARED
            ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/reader.cpp
            ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/writer.cpp)
    endif()

    add_dependencies(${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE} ${TGT_ARILES_BRIDGE_LIB})
    add_dependencies(${TGT_ARILES_BRIDGE_LIB} TGT_ariles_copy_headers)
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${TGT_ARILES_BRIDGE_LIB};${ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS}")
endif()

if (${ARILES_BRIDGE}_WITH_JSONNET)
    if (TARGET ${TGT_ARILES_BRIDGE_LIB})
        add_dependencies(${TGT_ARILES_BRIDGE_LIB} "${BRIDGE_TARGET_PREFIX}_jsonnet")
    else()
        add_dependencies(${TGT_RAPIDJSON} "${BRIDGE_TARGET_PREFIX}_jsonnet")
    endif()
endif()


include(ariles_install_component)
set(DEB_${ARILES_COMPONENT}_DEPENDS "rapidjson-dev")
