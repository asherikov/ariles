set (TGT_ARILES_BRIDGE_LIB      "ariles_visitor_${ARILES_BRIDGE}")

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
            find_path("ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES" NAMES rapidjson/rapidjson.h)
            if (NOT ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES)
                message(FATAL_ERROR "${ARILES_BRIDGE} is not found.")
            endif()
        endif()
    endif()


    include_directories(${CMAKE_CURRENT_LIST_DIR})

    link_directories(${ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS})
    add_library(${TGT_ARILES_BRIDGE_LIB} SHARED
        ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/reader.cpp
        ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/writer.cpp)
endif()

include(ariles_install_component)
set(DEB_${ARILES_COMPONENT}_DEPENDS "rapidjson-dev")
