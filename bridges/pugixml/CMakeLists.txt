cmakeut_copy_file_if_exists( TGT_ariles_copy_headers                             "bridge_pugixml_h"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/pugixml.h"       "${ARILES_BRIDGE_INCLUDE_DIR}/")
cmakeut_copy_dir_if_exists(  TGT_ariles_copy_headers                             "bridge_pugixml_dir"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/pugixml"         "${ARILES_BRIDGE_INCLUDE_DIR}/pugixml")
add_dependencies("TGT_ariles_copy_headers_bridge_pugixml_h" "TGT_ariles_copy_headers_bridge_pugixml_dir")
add_dependencies("TGT_ariles_copy_headers_bridge_pugixml_h" "TGT_ariles_copy_headers_ariles")


set (TGT_PUGIXML            "${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE}")
if(NOT ARILES_DEB_TARGETS)
    if(${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "ON")
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
