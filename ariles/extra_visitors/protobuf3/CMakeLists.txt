set (TGT_ARILES_VISITOR_LIB      "${PROJECT_NAME}_visitor_${ARILES_VISITOR}")

if(NOT ARILES_DEB_TARGETS)
    find_package(Protobuf REQUIRED)

    set(ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES "${Protobuf_INCLUDE_DIRS}")
    set(ARILES_VISITOR_${ARILES_VISITOR}_LIBS "${Protobuf_LIBRARIES}")
endif()

include(ariles_install_component)
