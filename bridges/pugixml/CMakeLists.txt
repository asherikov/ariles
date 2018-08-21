ariles_copy_file_if_exists( TGT_ariles_copy_headers                             "format_pugixml_h"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/pugixml.h"       "${ARILES_BRIDGE_INCLUDE_DIR}/")
ariles_copy_dir_if_exists(  TGT_ariles_copy_headers                             "format_pugixml_dir"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/pugixml"         "${ARILES_BRIDGE_INCLUDE_DIR}/pugixml")
add_dependencies("TGT_ariles_copy_headers_format_pugixml_h" "TGT_ariles_copy_headers_format_pugixml_dir")
add_dependencies("TGT_ariles_copy_headers_format_pugixml_h" "TGT_ariles_copy_headers_ariles")


set (TGT_PUGIXML            "${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE}")
if(${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "BUILD")
    set (PUGIXML_SOURCE_DIR     "${CMAKE_CURRENT_LIST_DIR}/pugixml")
    set (PUGIXML_CXX_FLAGS      "${ARILES_CXX_FLAGS} -fPIC")


    set(PUGIXML_ARGS
        ${BRIDGES_ARGS}
        "-DCMAKE_CXX_FLAGS=${PUGIXML_CXX_FLAGS}")

    if(ARILES_COPY_CXX_FLAGS_TO_C_FLAGS)
        set(PUGIXML_ARGS
            ${PUGIXML_ARGS}
            "-DCMAKE_C_FLAGS=${PUGIXML_CXX_FLAGS}")
    endif(ARILES_COPY_CXX_FLAGS_TO_C_FLAGS)

    ariles_add_external_git_project(TARGET_NAME "${TGT_PUGIXML}"
                                    PROJECT_DIR "${PUGIXML_SOURCE_DIR}"
                                    CMAKE_ARGS   ${PUGIXML_ARGS})

    set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "" PARENT_SCOPE)
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS
        "${ARILES_BRIDGES_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}pugixml${CMAKE_STATIC_LIBRARY_SUFFIX}" PARENT_SCOPE)
else()
    pkg_search_module(${ARILES_BRIDGE} REQUIRED pugixml)

    add_custom_target(${TGT_PUGIXML})
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS "${${ARILES_BRIDGE}_LIBRARY_DIRS}" PARENT_SCOPE)
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "${${ARILES_BRIDGE}_INCLUDE_DIRS}" PARENT_SCOPE)
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${${ARILES_BRIDGE}_LIBRARIES}" PARENT_SCOPE)
endif()
