set (TGT_ARILES_VISITOR_LIB      "${PROJECT_NAME}_visitor_${ARILES_VISITOR}")

if(NOT ARILES_DEB_TARGETS)
    set(protobuf_MODULE_COMPATIBLE ON CACHE STRING "")
    find_package(Protobuf CONFIG)

    if(Protobuf_FOUND)
        set(ARILES_VISITOR_${ARILES_VISITOR}_LIBS "protobuf::libprotobuf")
    else()
        find_package(Protobuf REQUIRED)

        set(ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES "${Protobuf_INCLUDE_DIRS}")
        set(ARILES_VISITOR_${ARILES_VISITOR}_LIBS "${Protobuf_LIBRARIES}")
    endif()
endif()

add_library(${TGT_ARILES_VISITOR_LIB} INTERFACE)

set(ARILES_COMPONENT_CMAKE_DEPS "Protobuf")
include(ariles_install_component)
