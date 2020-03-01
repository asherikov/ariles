list (FIND ARILES_ENABLED_BRIDGES "yaml_cpp" INDEX)
if (${INDEX} GREATER -1)
    message(FATAL_ERROR "Conflict between the bridges detected (yaml_cpp -- yaml_cpp03).")
endif()


#cmakeut_copy_file_if_exists( TGT_ariles_copy_headers                             "bridge_yaml_cpp03_h"
#                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/yaml_cpp03.h"    "${ARILES_BRIDGE_INCLUDE_DIR}/")
cmakeut_copy_dir_if_exists(  TGT_ariles_copy_headers                             "bridge_yaml_cpp03_dir"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/yaml_cpp03"      "${ARILES_BRIDGE_INCLUDE_DIR}/yaml_cpp03")
#add_dependencies("TGT_ariles_copy_headers_bridge_yaml_cpp03_h" "TGT_ariles_copy_headers_bridge_yaml_cpp03_dir")
#add_dependencies("TGT_ariles_copy_headers_bridge_yaml_cpp03_h" "TGT_ariles_copy_headers_ariles")
add_dependencies("TGT_ariles_copy_headers_bridge_yaml_cpp03_dir" "TGT_ariles_copy_headers_ariles")


set (TGT_YAMLCPP03            "${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE}")
if(NOT ARILES_DEB_TARGETS)
    if(${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "ON")
        pkg_search_module(${ARILES_BRIDGE} REQUIRED yaml-cpp03)

        add_custom_target(${TGT_YAMLCPP03})
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS "${${ARILES_BRIDGE}_LIBRARY_DIRS}")
        if (EXISTS "${${ARILES_BRIDGE}_INCLUDE_DIRS}/yaml-cpp03")
            set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDE_HEADER "yaml-cpp03/yaml.h")
        else()
            set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDE_HEADER "yaml-cpp/yaml.h")
        endif()
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "${${ARILES_BRIDGE}_INCLUDE_DIRS}")
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${${ARILES_BRIDGE}_LIBRARIES}")
    endif()
endif()

include(ariles_install_component)

configure_file("${${ARILES_BRIDGE}_INTERFACE_DIR}/yaml_cpp03.h" "${ARILES_BRIDGE_INCLUDE_DIR}/yaml_cpp03.h")
