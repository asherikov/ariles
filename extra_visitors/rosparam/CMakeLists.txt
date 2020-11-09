set (TGT_ARILES_VISITOR_LIB      "${PROJECT_NAME}_visitor_${ARILES_VISITOR}")

add_library(${TGT_ARILES_VISITOR_LIB} SHARED
    ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/reader.cpp
    ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/writer.cpp
)

if(NOT ARILES_DEB_TARGETS)
    find_package(roscpp  REQUIRED)

    set(ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES "${roscpp_INCLUDE_DIRS}")
    set(ARILES_VISITOR_${ARILES_VISITOR}_LIBS "${roscpp_LIBRARIES}")

    target_include_directories(${TGT_ARILES_VISITOR_LIB} SYSTEM PUBLIC
        ${ARILES_VISITOR_${ARILES_VISITOR}_INCLUDES}
    )

    target_link_libraries(${TGT_ARILES_VISITOR_LIB} PUBLIC
        ${ARILES_VISITOR_${ARILES_VISITOR}_LIBS}
    )
endif()

include(ariles_install_component)
set(ARILES_ROS_DISTRO "${ARILES_VISITOR_rosparam}")
set(DEB_${ARILES_COMPONENT}_DEPENDS "ros-${ARILES_ROS_DISTRO}-ros-base")
