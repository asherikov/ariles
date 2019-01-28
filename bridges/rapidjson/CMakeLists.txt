cmakeut_copy_file_if_exists( TGT_ariles_copy_headers                             "bridge_rapidjson_h"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/rapidjson.h"     "${ARILES_BRIDGE_INCLUDE_DIR}/")
cmakeut_copy_dir_if_exists(  TGT_ariles_copy_headers                             "bridge_rapidjson_dir"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/rapidjson"       "${ARILES_BRIDGE_INCLUDE_DIR}/rapidjson")
add_dependencies("TGT_ariles_copy_headers_bridge_rapidjson_h" "TGT_ariles_copy_headers_bridge_rapidjson_dir")
add_dependencies("TGT_ariles_copy_headers_bridge_rapidjson_h" "TGT_ariles_copy_headers_ariles")

set(RAPIDJSON_SOURCE_DIR     "${CMAKE_CURRENT_LIST_DIR}/rapidjson")

set (TGT_RAPIDJSON            "${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE}")
if(NOT ARILES_DEB_TARGETS)
    if(${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "BUILD")
        set (RAPIDJSON_CXX_FLAGS      "${ARILES_CXX_FLAGS} -fPIC")


        set(RAPIDJSON_ARGS
            ${BRIDGES_ARGS}
            -DRAPIDJSON_BUILD_DOC=OFF
            -DRAPIDJSON_BUILD_EXAMPLES=OFF
            -DRAPIDJSON_BUILD_TESTS=OFF
            -DRAPIDJSON_BUILD_CXX11=OFF
            -DRAPIDJSON_ENABLE_INSTRUMENTATION_OPT=OFF
            -DRAPIDJSON_HAS_STDSTRING=ON
            "-DCMAKE_CXX_FLAGS=${RAPIDJSON_CXX_FLAGS}")

        if(ARILES_COPY_CXX_FLAGS_TO_C_FLAGS)
            set(RAPIDJSON_ARGS
                ${RAPIDJSON_ARGS}
                "-DCMAKE_C_FLAGS=${RAPIDJSON_CXX_FLAGS}")
        endif(ARILES_COPY_CXX_FLAGS_TO_C_FLAGS)

        cmakeut_add_external_git_project(TARGET_NAME "${TGT_RAPIDJSON}"
                                        PROJECT_DIR "${RAPIDJSON_SOURCE_DIR}"
                                        CMAKE_ARGS   ${RAPIDJSON_ARGS})

        set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "")
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "")
    else()
        pkg_search_module(${ARILES_BRIDGE} RapidJSON)

        if (${ARILES_BRIDGE}_FOUND)
            add_custom_target(${TGT_RAPIDJSON})
            set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS "${${ARILES_BRIDGE}_LIBRARY_DIRS}")
            set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "${${ARILES_BRIDGE}_INCLUDE_DIRS}")
            set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${${ARILES_BRIDGE}_LIBRARIES}")
        else()
            add_custom_target(${TGT_RAPIDJSON})

            set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS "")
            set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "")

            find_path("ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES" NAMES rapidjson/rapidjson.hpp)
            if (${ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES}-NOTFOUND)
                message(FATAL_ERROR "${ARILES_BRIDGE} is not found.")
            endif()
        endif()
    endif()
endif()

list (FIND ARILES_ENABLED_BRIDGES "jsonnet" INDEX)
if (${INDEX} GREATER -1)
    add_dependencies(${TGT_RAPIDJSON} "${BRIDGE_TARGET_PREFIX}_jsonnet")
endif()

include(ariles_install_component)
set(DEB_${ARILES_COMPONENT}_DEPENDS "rapidjson-dev")
