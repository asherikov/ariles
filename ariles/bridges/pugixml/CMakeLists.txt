cmakeut_copy_file_if_exists( TGT_ariles_copy_headers                             "bridge_pugixml_h"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/pugixml.h"       "${ARILES_BRIDGE_INCLUDE_DIR}/")
cmakeut_copy_dir_if_exists(  TGT_ariles_copy_headers                             "bridge_pugixml_dir"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/pugixml"         "${ARILES_BRIDGE_INCLUDE_DIR}/pugixml")
add_dependencies("TGT_ariles_copy_headers_bridge_pugixml_h" "TGT_ariles_copy_headers_bridge_pugixml_dir")
add_dependencies("TGT_ariles_copy_headers_bridge_pugixml_h" "TGT_ariles_copy_headers_ariles")

set(PUGIXML_SOURCE_DIR     "${CMAKE_CURRENT_LIST_DIR}/pugixml")

set (TGT_PUGIXML            "${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE}")
if(NOT ARILES_DEB_TARGETS)
    if(${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "BUILD")
        set (PUGIXML_CXX_FLAGS      "${ARILES_CXX_FLAGS} -fPIC")


        set(PUGIXML_ARGS
            ${BRIDGES_ARGS}
            "-DCMAKE_CXX_FLAGS=${PUGIXML_CXX_FLAGS}")

        if(ARILES_COPY_CXX_FLAGS_TO_C_FLAGS)
            set(PUGIXML_ARGS
                ${PUGIXML_ARGS}
                "-DCMAKE_C_FLAGS=${PUGIXML_CXX_FLAGS}")
        endif(ARILES_COPY_CXX_FLAGS_TO_C_FLAGS)

        cmakeut_add_external_git_project(TARGET_NAME "${TGT_PUGIXML}"
                                        PROJECT_DIR "${PUGIXML_SOURCE_DIR}"
                                        CMAKE_ARGS   ${PUGIXML_ARGS})

        set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "")
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS
            "${ARILES_BRIDGES_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}pugixml${CMAKE_STATIC_LIBRARY_SUFFIX}")
    else()
        pkg_search_module(${ARILES_BRIDGE} pugixml)


        if (${ARILES_BRIDGE}_FOUND)
            add_custom_target(${TGT_PUGIXML})
            set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS "${${ARILES_BRIDGE}_LIBRARY_DIRS}")
            set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "${${ARILES_BRIDGE}_INCLUDE_DIRS}")
            set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${${ARILES_BRIDGE}_LIBRARIES}")
        else()
            add_custom_target(${TGT_PUGIXML})

            set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS "")

            find_library("ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS" NAMES ${ARILES_BRIDGE})
            if (${ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS}-NOTFOUND)
                message(FATAL_ERROR "${ARILES_BRIDGE} is not found.")
            endif()

            find_path("ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES" NAMES pugixml.hpp)
            if (${ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES}-NOTFOUND)
                message(FATAL_ERROR "${ARILES_BRIDGE} is not found.")
            endif()
        endif()
    endif()
endif()

include(ariles_install_component)
set(DEB_${ARILES_COMPONENT}_DEPENDS "libpugixml-dev")
