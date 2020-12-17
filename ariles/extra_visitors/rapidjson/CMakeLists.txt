set (TGT_ARILES_VISITOR_LIB      "${PROJECT_NAME}_visitor_${ARILES_VISITOR}")

if(NOT ARILES_DEB_TARGETS)
    find_path("ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES" NAMES rapidjson/rapidjson.hpp)
    if (NOT ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES)
        find_path("ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES" NAMES rapidjson/rapidjson.h)
        if (NOT ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES)
            pkg_search_module(${ARILES_VISITOR} RapidJSON)

            if (${ARILES_VISITOR}_FOUND)
                set(ARILES_VISITOR_${ARILES_VISITOR}_LIBRARY_DIRS "${${ARILES_VISITOR}_LIBRARY_DIRS}")
                set(ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES "${${ARILES_VISITOR}_INCLUDE_DIRS}")
                set(ARILES_VISITOR_${ARILES_VISITOR}_LIBS "${${ARILES_VISITOR}_LIBRARIES}")

                link_directories(${ARILES_VISITOR_${ARILES_VISITOR}_LIBRARY_DIRS})
            else()
                message(FATAL_ERROR "${ARILES_VISITOR} is not found.")
            endif()
        endif()
    endif()
endif()

add_library(${TGT_ARILES_VISITOR_LIB} SHARED
    ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/reader.cpp
    ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/writer.cpp
)

include(ariles_install_component)
set(DEB_${ARILES_COMPONENT}_DEPENDS "rapidjson-dev")
