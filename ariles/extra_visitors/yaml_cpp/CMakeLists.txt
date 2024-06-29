set (TGT_ARILES_VISITOR_LIB      "${PROJECT_NAME}_visitor_${ARILES_VISITOR}")

if(NOT ARILES_DEB_TARGETS)
    find_package(yaml-cpp)

    if(yaml-cpp_FOUND)
        set(ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES "${YAML_CPP_INCLUDE_DIR}")
        if(TARGET yaml-cpp)
            set(ARILES_VISITOR_${ARILES_VISITOR}_LIBS "yaml-cpp")
        else()
            set(ARILES_VISITOR_${ARILES_VISITOR}_LIBS "yaml-cpp::yaml-cpp")
        endif()
    else()
        pkg_search_module(${ARILES_VISITOR} REQUIRED yaml-cpp>=0.5.1)

        set(ARILES_VISITOR_${ARILES_VISITOR}_LIBRARY_DIRS "${${ARILES_VISITOR}_LIBRARY_DIRS}")
        set(ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES "${${ARILES_VISITOR}_INCLUDE_DIRS}")
        set(ARILES_VISITOR_${ARILES_VISITOR}_LIBS "${${ARILES_VISITOR}_LIBRARIES}")

        link_directories(${ARILES_VISITOR_${ARILES_VISITOR}_LIBRARY_DIRS})
    endif()
endif()

add_library(${TGT_ARILES_VISITOR_LIB}
    ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/reader.cpp
    ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/writer.cpp
)

include(ariles_install_component)
set(DEB_${ARILES_COMPONENT}_DEPENDS "libyaml-cpp-dev")
