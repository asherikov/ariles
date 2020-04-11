set (TGT_ARILES_BRIDGE_LIB  "ariles_visitor_${ARILES_BRIDGE}")
if(NOT ARILES_DEB_TARGETS)
    find_package(roscpp  REQUIRED)

    set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "${roscpp_INCLUDE_DIRS}")
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${roscpp_LIBRARIES}")


    include_directories(${CMAKE_CURRENT_LIST_DIR})

    link_directories(${ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS})
    add_library(${TGT_ARILES_BRIDGE_LIB} SHARED
        ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/reader.cpp
        ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/writer.cpp)
endif()


include(ariles_install_component)
set(DEB_${ARILES_COMPONENT}_DEPENDS "ros-${ARILES_BRIDGE_ros}-ros-base")
