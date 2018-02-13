set (TGT_MESSAGEPACK            "${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE}")
set (MESSAGEPACK_SOURCE_DIR     "${CMAKE_CURRENT_LIST_DIR}/msgpack-c")
set (MESSAGEPACK_INTERFACE_DIR  "${CMAKE_CURRENT_LIST_DIR}/interface")
set (MESSAGEPACK_CXX_FLAGS      "-fPIC")


set(MESSAGEPACK_ARGS
    ${BRIDGES_ARGS}
    -DMSGPACK_USE_X3_PARSE=OFF
    -DMSGPACK_CXX17=OFF
    -DMSGPACK_CXX11=OFF
    -DMSGPACK_32BIT=OFF
    -DMSGPACK_BOOST=OFF
    -DMSGPACK_BUILD_EXAMPLES=OFF
    -DMSGPACK_BUILD_TESTS=OFF
    -DMSGPACK_ENABLE_CXX=ON
    -DMSGPACK_ENABLE_SHARED=OFF
    "-DCMAKE_CXX_FLAGS=${MESSAGEPACK_CXX_FLAGS}")

if(ARILES_COPY_CXX_FLAGS_TO_C_FLAGS)
    set(MESSAGEPACK_ARGS
        ${MESSAGEPACK_ARGS}
        "-DCMAKE_C_FLAGS=${MESSAGEPACK_CXX_FLAGS}")
endif(ARILES_COPY_CXX_FLAGS_TO_C_FLAGS)

ariles_add_external_git_project(TARGET_NAME "${TGT_MESSAGEPACK}"
                                PROJECT_DIR "${MESSAGEPACK_SOURCE_DIR}"
                                CMAKE_ARGS   ${MESSAGEPACK_ARGS})

ariles_copy_file_if_exists( TGT_ariles_copy_headers                         "format_msgpack_h"
                            "${MESSAGEPACK_INTERFACE_DIR}/format_msgpack.h" "${ARILES_INCLUDE_DIR}/ariles/")
ariles_copy_dir_if_exists(  TGT_ariles_copy_headers                         "format_msgpack_dir"
                            "${MESSAGEPACK_INTERFACE_DIR}/format_msgpack"   "${ARILES_INCLUDE_DIR}/ariles/format_msgpack")
add_dependencies("TGT_ariles_copy_headers_format_msgpack_h" "TGT_ariles_copy_headers_ariles")

set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "" PARENT_SCOPE)
set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS
    "${ARILES_BRIDGES_LIBDIR}/${CMAKE_SHARED_MODULE_PREFIX}msgpackc${CMAKE_STATIC_LIBRARY_SUFFIX}" PARENT_SCOPE)
