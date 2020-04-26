set (TGT_ARILES_VISITOR_LIB  "ariles_visitor_${ARILES_VISITOR}")
if(NOT ARILES_DEB_TARGETS)
    find_package(roscpp  REQUIRED)

    set(ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES "${roscpp_INCLUDE_DIRS}")
    set(ARILES_VISITOR_${ARILES_VISITOR}_LIBS "${roscpp_LIBRARIES}")


    link_directories(${ARILES_VISITOR_${ARILES_VISITOR}_LIBRARY_DIRS})
    add_library(${TGT_ARILES_VISITOR_LIB} SHARED
        ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/reader.cpp
        ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/writer.cpp)
    target_include_directories(${TGT_ARILES_VISITOR_LIB} SYSTEM PUBLIC ${ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES})
endif()


include(ariles_install_component)
set(DEB_${ARILES_COMPONENT}_DEPENDS "ros-${ARILES_VISITOR_ros}-ros-base")
