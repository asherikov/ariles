foreach(ARILES_COMPONENT  core yaml-cpp octave ros array)
    find_package(ariles-${ARILES_COMPONENT} REQUIRED)

    list(APPEND ariles_ros_LIBRARIES     "${ariles-${ARILES_COMPONENT}_LIBRARIES}")
    list(APPEND ariles_ros_INCLUDE_DIRS  "${ariles-${ARILES_COMPONENT}_INCLUDE_DIRS}")
endforeach()
