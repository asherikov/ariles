cmakeut_copy_file_if_exists( TGT_ariles_copy_headers                         "bridge_msgpack_h"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/msgpack.h"   "${ARILES_BRIDGE_INCLUDE_DIR}/")
cmakeut_copy_dir_if_exists(  TGT_ariles_copy_headers                         "bridge_msgpack_dir"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/msgpack"     "${ARILES_BRIDGE_INCLUDE_DIR}/msgpack")
add_dependencies("TGT_ariles_copy_headers_bridge_msgpack_h" "TGT_ariles_copy_headers_bridge_msgpack_dir")
add_dependencies("TGT_ariles_copy_headers_bridge_msgpack_h" "TGT_ariles_copy_headers_ariles")


set (TGT_MESSAGEPACK            "${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE}")
if(NOT ARILES_DEB_TARGETS)
    if(${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "ON")
        pkg_search_module(${ARILES_BRIDGE} msgpack)

        if (${ARILES_BRIDGE}_FOUND)
            add_custom_target(${TGT_MESSAGEPACK})
            set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS "${${ARILES_BRIDGE}_LIBRARY_DIRS}")
            set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "${${ARILES_BRIDGE}_INCLUDE_DIRS}")
            set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${${ARILES_BRIDGE}_LIBRARIES}")
        else()
            add_custom_target(${TGT_MESSAGEPACK})

            find_library("ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS" NAMES ${ARILES_BRIDGE})
            if (${ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS}-NOTFOUND)
                message(FATAL_ERROR "${ARILES_BRIDGE} is not found.")
            endif()

            find_path("ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES" NAMES msgpack.hpp)
            if (${ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES}-NOTFOUND)
                message(FATAL_ERROR "${ARILES_BRIDGE} is not found.")
            endif()
        endif()
    endif()
endif()

include(ariles_install_component)
