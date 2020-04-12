set (TGT_ARILES_BRIDGE_LIB  "ariles_visitor_${ARILES_BRIDGE}")
if(NOT ARILES_DEB_TARGETS)
    pkg_search_module(${ARILES_BRIDGE} REQUIRED yaml-cpp>=0.5.1)


    set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS "${${ARILES_BRIDGE}_LIBRARY_DIRS}")
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "${${ARILES_BRIDGE}_INCLUDE_DIRS}")
    set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${${ARILES_BRIDGE}_LIBRARIES}")


    include_directories(${CMAKE_CURRENT_LIST_DIR})

    link_directories(${ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS})
    add_library(${TGT_ARILES_BRIDGE_LIB} SHARED
        ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/reader.cpp
        ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/src/writer.cpp)
endif()


include(ariles_install_component)
set(DEB_${ARILES_COMPONENT}_DEPENDS "libyaml-cpp-dev")
