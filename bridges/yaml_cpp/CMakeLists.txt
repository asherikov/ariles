list (FIND ARILES_ENABLED_BRIDGES "yaml_cpp03" INDEX)
if (${INDEX} GREATER -1)
    message(FATAL_ERROR "Conflict between the bridges detected (yaml_cpp03 -- yaml_cpp).")
endif()


cmakeut_copy_file_if_exists( TGT_ariles_copy_headers                             "bridge_yaml_cpp_h"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/yaml_cpp.h"      "${ARILES_BRIDGE_INCLUDE_DIR}/")
cmakeut_copy_dir_if_exists(  TGT_ariles_copy_headers                             "bridge_yaml_cpp_dir"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/yaml_cpp"        "${ARILES_BRIDGE_INCLUDE_DIR}/yaml_cpp")
add_dependencies("TGT_ariles_copy_headers_bridge_yaml_cpp_h" "TGT_ariles_copy_headers_bridge_yaml_cpp_dir")
add_dependencies("TGT_ariles_copy_headers_bridge_yaml_cpp_h" "TGT_ariles_copy_headers_ariles")

set(YAMLCPP_SOURCE_DIR     "${CMAKE_CURRENT_LIST_DIR}/yaml-cpp")

set (TGT_YAMLCPP            "${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE}")
if(NOT ARILES_DEB_TARGETS)
    if(${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "BUILD")
        set (YAMLCPP_CXX_FLAGS      "${ARILES_CXX_FLAGS} -fPIC")


        set(YAMLCPP_ARGS
            ${BRIDGES_ARGS}
            -DYAML_CPP_BUILD_TESTS=OFF
            -DYAML_CPP_BUILD_TOOLS=OFF
            -DYAML_CPP_BUILD_CONTRIB=OFF
            "-DCMAKE_CXX_FLAGS=${YAMLCPP_CXX_FLAGS}")

        if(ARILES_COPY_CXX_FLAGS_TO_C_FLAGS)
            set(YAMLCPP_ARGS
                ${YAMLCPP_ARGS}
                "-DCMAKE_C_FLAGS=${YAMLCPP_CXX_FLAGS}")
        endif(ARILES_COPY_CXX_FLAGS_TO_C_FLAGS)

        cmakeut_add_external_git_project(TARGET_NAME "${TGT_YAMLCPP}"
                                        PROJECT_DIR "${YAMLCPP_SOURCE_DIR}"
                                        CMAKE_ARGS   ${YAMLCPP_ARGS})

        set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "")
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS
            "${ARILES_BRIDGES_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}yaml-cpp${CMAKE_STATIC_LIBRARY_SUFFIX}")
    else()
        pkg_search_module(${ARILES_BRIDGE} REQUIRED yaml-cpp>=0.5.1)

        add_custom_target(${TGT_YAMLCPP})
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS "${${ARILES_BRIDGE}_LIBRARY_DIRS}")
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "${${ARILES_BRIDGE}_INCLUDE_DIRS}")
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${${ARILES_BRIDGE}_LIBRARIES}")
    endif()
endif()

include(ariles_install_component)
set(DEB_${ARILES_COMPONENT}_DEPENDS "libyaml-cpp-dev")
