ariles_copy_file_if_exists( TGT_ariles_copy_headers                             "format_yaml_h"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/format_yaml.h"   "${ARILES_INCLUDE_DIR}/ariles/")
ariles_copy_dir_if_exists(  TGT_ariles_copy_headers                             "format_yaml_dir"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/format_yaml"     "${ARILES_INCLUDE_DIR}/ariles/format_yaml")
add_dependencies("TGT_ariles_copy_headers_format_yaml_h" "TGT_ariles_copy_headers_ariles")


set (TGT_YAMLCPP            "${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE}")
if(${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "BUILD")
    set (YAMLCPP_SOURCE_DIR     "${CMAKE_CURRENT_LIST_DIR}/yaml-cpp")
    set (YAMLCPP_CXX_FLAGS      "${ARILES_CXX_FLAGS} -fPIC")


    set(YAMLCPP_ARGS
        ${BRIDGES_ARGS}
        -DYAML_CPP_BUILD_TOOLS=OFF
        -DYAML_CPP_BUILD_CONTRIB=OFF
        "-DCMAKE_CXX_FLAGS=${YAMLCPP_CXX_FLAGS}")

    if(ARILES_COPY_CXX_FLAGS_TO_C_FLAGS)
        set(YAMLCPP_ARGS
            ${YAMLCPP_ARGS}
            "-DCMAKE_C_FLAGS=${YAMLCPP_CXX_FLAGS}")
    endif(ARILES_COPY_CXX_FLAGS_TO_C_FLAGS)

    ariles_add_external_git_project(TARGET_NAME "${TGT_YAMLCPP}"
                                    PROJECT_DIR "${YAMLCPP_SOURCE_DIR}"
                                    CMAKE_ARGS   ${YAMLCPP_ARGS})

    set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "" PARENT_SCOPE)
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS
        "${ARILES_BRIDGES_LIBDIR}/${CMAKE_SHARED_MODULE_PREFIX}yaml-cpp${CMAKE_STATIC_LIBRARY_SUFFIX}" PARENT_SCOPE)
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDE_HEADER "yaml-cpp/yaml.h" PARENT_SCOPE)
else()
    pkg_check_modules(${ARILES_BRIDGE} REQUIRED yaml-cpp03)

    add_custom_target(${TGT_YAMLCPP})
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS "${${ARILES_BRIDGE}_LIBRARY_DIRS}" PARENT_SCOPE)
    if (EXISTS "${${ARILES_BRIDGE}_INCLUDE_DIRS}/yaml-cpp03")
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDE_HEADER "yaml-cpp03/yaml.h" PARENT_SCOPE)
    else()
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDE_HEADER "yaml-cpp/yaml.h" PARENT_SCOPE)
    endif()
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "${${ARILES_BRIDGE}_INCLUDE_DIRS}" PARENT_SCOPE)
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${${ARILES_BRIDGE}_LIBRARIES}" PARENT_SCOPE)
endif()
