function(ariles_install_component ARILES_BRIDGE)
    string(REGEX REPLACE "_" "-" ARILES_COMPONENT "${ARILES_BRIDGE}")
    set(ARILES_COMPONENT "${ARILES_COMPONENT}" PARENT_SCOPE)

    cpack_add_component(
        "${ARILES_COMPONENT}"
        DISPLAY_NAME "'${ARILES_BRIDGE}' support for ariles"
        DESCRIPTION "Enables support for '${ARILES_BRIDGE}' in ariles"
        DEPENDS "core"
        ARCHIVE_FILE "${CPACK_PACKAGE_NAME}-${ARILES_COMPONENT_NAME}-${CPACK_PACKAGE_VERSION}")


    set(ariles_LIBRARIES "ariles-${ARILES_COMPONENT}_LIBRARIES")
    set(ariles_INCLUDE_DIRS "ariles-${ARILES_COMPONENT}_INCLUDE_DIRS")
    set(ariles_LIBRARY_DIRS "ariles-${ARILES_COMPONENT}_LIBRARY_DIRS")

    set(ARILES_LIBRARIES            "${ARILES_COMPONENT_${ARILES_COMPONENT}_LIBS}")
    set(ARILES_INCLUDES             "")
    set(ARILES_DEPENDENCY_INCLUDES  "${ARILES_COMPONENT_${ARILES_COMPONENT}_INCLUDES}")
    set(ARILES_LIBRARY_DIRS         "${ARILES_COMPONENT_${ARILES_COMPONENT}_LIBRARY_DIRS}")

    if (ARILES_LIBRARIES)
        set (ARILES_SHARED_GNU_LINKER_FLAGS "-Wl,--exclude-libs")

        foreach(ARILES_LIBRARY ${ARILES_LIBRARIES})
            set(ARILES_SHARED_GNU_LINKER_FLAGS "${ARILES_SHARED_GNU_LINKER_FLAGS},${ARILES_LIBRARY}")
        endforeach(ARILES_LIBRARY)
    endif (ARILES_LIBRARIES)

    list(REMOVE_DUPLICATES ARILES_INCLUDES)
    list(REMOVE_DUPLICATES ARILES_DEPENDENCY_INCLUDES)
    list(REMOVE_DUPLICATES ARILES_LIBRARIES)
    list(REMOVE_ITEM ARILES_INCLUDES "")
    list(REMOVE_ITEM ARILES_DEPENDENCY_INCLUDES "")
    list(REMOVE_ITEM ARILES_LIBRARIES "")

    if (ARILES_INCLUDES)
        configure_package_config_file(  "${PROJECT_SOURCE_DIR}/cmake/arilesConfig.cmake.in"
                                        "${PROJECT_BINARY_DIR}/ariles-${ARILES_COMPONENT}-Config.cmake"
                                        INSTALL_DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ariles-${ARILES_COMPONENT}/"
                                        PATH_VARS ARILES_INCLUDES
                                        NO_SET_AND_CHECK_MACRO
                                        NO_CHECK_REQUIRED_COMPONENTS_MACRO)
    else()
        configure_package_config_file(  "${PROJECT_SOURCE_DIR}/cmake/arilesConfig.cmake.in"
                                        "${PROJECT_BINARY_DIR}/ariles-${ARILES_COMPONENT}Config.cmake"
                                        INSTALL_DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ariles-${ARILES_COMPONENT}/"
                                        NO_SET_AND_CHECK_MACRO
                                        NO_CHECK_REQUIRED_COMPONENTS_MACRO)
    endif()

    install(FILES "${PROJECT_BINARY_DIR}/ariles-${ARILES_COMPONENT}Config.cmake"
            DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ariles-${ARILES_COMPONENT}/"
            COMPONENT ${ARILES_COMPONENT})

    write_basic_package_version_file(
        ${PROJECT_BINARY_DIR}/ariles-${ARILES_COMPONENT}ConfigVersion.cmake
        VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}
        COMPATIBILITY SameMajorVersion)
    install(FILES "${PROJECT_BINARY_DIR}/ariles-${ARILES_COMPONENT}ConfigVersion.cmake"
            DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ariles/"
            COMPONENT ${ARILES_COMPONENT})

    if(ARILES_PKGCONFIG_INSTALL_PATH)
        string(REPLACE ";" " -I" ARILES_INCLUDES_FLAGS "${ARILES_INCLUDES}")
        set(ARILES_INCLUDES_FLAGS "-I${ARILES_INCLUDES_FLAGS}")
        string(REPLACE ";" " -l" ARILES_LIBRARIES_FLAGS "${ARILES_LIBRARIES}")
        string(REPLACE ";" " -L" ARILES_LIBRARIES_FLAGS_DIRS "${ARILES_LIBRARY_DIRS}")
        set(ARILES_LIBRARIES_FLAGS "-l${ARILES_LIBRARIES_FLAGS} -L${ARILES_LIBRARIES_FLAGS_DIRS}")
        configure_file("cmake/ariles.pc.in"             "${PROJECT_BINARY_DIR}/ariles-${ARILES_COMPONENT}.pc" @ONLY)

        install(FILES "${PROJECT_BINARY_DIR}/ariles-${ARILES_COMPONENT}.pc"
                DESTINATION "${ARILES_PKGCONFIG_INSTALL_PATH}"
                COMPONENT ${ARILES_COMPONENT})
    endif()

    install (DIRECTORY "${ARILES_INCLUDE_DIR}/${ARILES_BRIDGE}/"
             DESTINATION "${CMAKE_INSTALL_PREFIX}/include/"
             COMPONENT "${ARILES_COMPONENT}")
endfunction()