ariles_copy_file_if_exists( TGT_ariles_copy_headers                             "format_rapidjson_h"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/rapidjson.h"     "${ARILES_BRIDGE_INCLUDE_DIR}/")
ariles_copy_dir_if_exists(  TGT_ariles_copy_headers                             "format_rapidjson_dir"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/rapidjson"       "${ARILES_BRIDGE_INCLUDE_DIR}/rapidjson")
add_dependencies("TGT_ariles_copy_headers_format_rapidjson_h" "TGT_ariles_copy_headers_format_rapidjson_dir")
add_dependencies("TGT_ariles_copy_headers_format_rapidjson_h" "TGT_ariles_copy_headers_ariles")


set (TGT_RAPIDJSON            "${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE}")
if(${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "BUILD")
    set (RAPIDJSON_SOURCE_DIR     "${CMAKE_CURRENT_LIST_DIR}/rapidjson")
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

    ariles_add_external_git_project(TARGET_NAME "${TGT_RAPIDJSON}"
                                    PROJECT_DIR "${RAPIDJSON_SOURCE_DIR}"
                                    CMAKE_ARGS   ${RAPIDJSON_ARGS})

    set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "" PARENT_SCOPE)
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "" PARENT_SCOPE)
else()
    pkg_search_module(${ARILES_BRIDGE} REQUIRED RapidJSON)

    add_custom_target(${TGT_RAPIDJSON})
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS "${${ARILES_BRIDGE}_LIBRARY_DIRS}" PARENT_SCOPE)
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "${${ARILES_BRIDGE}_INCLUDE_DIRS}" PARENT_SCOPE)
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${${ARILES_BRIDGE}_LIBRARIES}" PARENT_SCOPE)
endif()
