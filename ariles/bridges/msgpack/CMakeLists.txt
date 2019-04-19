cmakeut_copy_file_if_exists( TGT_ariles_copy_headers                         "bridge_msgpack_h"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/msgpack.h"   "${ARILES_BRIDGE_INCLUDE_DIR}/")
cmakeut_copy_dir_if_exists(  TGT_ariles_copy_headers                         "bridge_msgpack_dir"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/msgpack"     "${ARILES_BRIDGE_INCLUDE_DIR}/msgpack")
add_dependencies("TGT_ariles_copy_headers_bridge_msgpack_h" "TGT_ariles_copy_headers_bridge_msgpack_dir")
add_dependencies("TGT_ariles_copy_headers_bridge_msgpack_h" "TGT_ariles_copy_headers_ariles")

set(MESSAGEPACK_SOURCE_DIR   "${CMAKE_CURRENT_LIST_DIR}/msgpack-c")

set (TGT_MESSAGEPACK            "${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE}")
if(NOT ARILES_DEB_TARGETS)
    if(${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "BUILD")
        set (MESSAGEPACK_CXX_FLAGS      "${ARILES_CXX_FLAGS} -fPIC")


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

        cmakeut_add_external_git_project(TARGET_NAME "${TGT_MESSAGEPACK}"
                                        PROJECT_DIR "${MESSAGEPACK_SOURCE_DIR}"
                                        CMAKE_ARGS   ${MESSAGEPACK_ARGS})

        set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "")
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS
            "${ARILES_BRIDGES_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}msgpackc${CMAKE_STATIC_LIBRARY_SUFFIX}")
    else()
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
