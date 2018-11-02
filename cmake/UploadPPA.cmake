##
# Copyright (c) 2010 Daniel Pfeifer <daniel@pfeifer-mail.de>
#               2011-2014 Stefan Eilemann <eile@eyescale.ch>
#               (https://github.com/Eyescale/CMake)
#               2018 Alexander Sherikov
#
#  sudo apt-get install devscripts
#
# set(PPA_BUILDPACKAGE_FLAGS "-us;-uc") disables signature
##


find_program(BUILDPACKAGE_EXECUTABLE dpkg-buildpackage)
find_program(DPUT_EXECUTABLE dput)
find_program(LINTIAN_EXECUTABLE lintian)

if(NOT BUILDPACKAGE_EXECUTABLE)
    message(FATAL_ERROR "dpkg-buildpackage not found")
endif()
if(NOT CPACK_PACKAGE_NAME)
    message(FATAL_ERROR "CPACK_PACKAGE_NAME not set")
endif()
if(NOT PPA_DPUT_HOST)
    message("PPA_DPUT_HOST not set")
else()
    if(NOT DPUT_EXECUTABLE)
        message(FATAL_ERROR "dput not found")
    endif()
endif()


# DEBIAN/control
# debian policy enforce lower case for package name
# Package: (mandatory)
IF(NOT CPACK_DEBIAN_PACKAGE_NAME)
    STRING(TOLOWER "${CPACK_PACKAGE_NAME}" CPACK_DEBIAN_PACKAGE_NAME)
ENDIF(NOT CPACK_DEBIAN_PACKAGE_NAME)

# Section: (recommended)
IF(NOT CPACK_DEBIAN_PACKAGE_SECTION)
    SET(CPACK_DEBIAN_PACKAGE_SECTION "devel")
ENDIF(NOT CPACK_DEBIAN_PACKAGE_SECTION)

# Priority: (recommended)
IF(NOT CPACK_DEBIAN_PACKAGE_PRIORITY)
    SET(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
ENDIF(NOT CPACK_DEBIAN_PACKAGE_PRIORITY)

file(STRINGS ${CPACK_PACKAGE_DESCRIPTION_FILE} DESC_LINES)
foreach(LINE ${DESC_LINES})
    set(DEB_LONG_DESCRIPTION "${DEB_LONG_DESCRIPTION} ${LINE}\n")
endforeach(LINE ${DESC_LINES})


set(PPA_MAIN_TARGET "ppa")
add_custom_target(${PPA_MAIN_TARGET})
foreach(UBUNTU_NAME ${PPA_UBUNTU_CODENAMES})
    set(PPA_UBUNTU_NAME_TARGET "ppa_${UBUNTU_NAME}")

    add_custom_target(${PPA_UBUNTU_NAME_TARGET})
    add_dependencies(${PPA_MAIN_TARGET}   "${PPA_UBUNTU_NAME_TARGET}")


    set(DEBIAN_BASE_DIR ${PROJECT_BINARY_DIR}/Debian/${UBUNTU_NAME})
    set(DEBIAN_SOURCE_DIR
        ${DEBIAN_BASE_DIR}/${CPACK_DEBIAN_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-source)
    set(DEBIAN_CONTROL ${DEBIAN_SOURCE_DIR}/debian/control)
    set(DEBIAN_COPYRIGHT ${DEBIAN_SOURCE_DIR}/debian/copyright)
    set(DEBIAN_RULES ${DEBIAN_SOURCE_DIR}/debian/rules)
    set(DEBIAN_CHANGELOG ${DEBIAN_SOURCE_DIR}/debian/changelog)
    set(DEBIAN_SOURCE_CHANGES
        ${CPACK_DEBIAN_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}~${UBUNTU_NAME}_source.changes)
    set(PPA_UBUNTU_NAME_BUILD_DEPENDS "cmake")
    set(PPA_UBUNTU_NAME_DEPENDS "")



    execute_process(COMMAND ${CMAKE_COMMAND} -E remove_directory "${DEBIAN_BASE_DIR}")
    execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${DEBIAN_BASE_DIR})

    execute_process(COMMAND ${CMAKE_COMMAND} -E tar cvf ${DEBIAN_BASE_DIR}.tar ${PPA_ARCHIVE_FILES}
                    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

    execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory "${DEBIAN_SOURCE_DIR}")

    execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf ${DEBIAN_BASE_DIR}.tar
                    WORKING_DIRECTORY ${DEBIAN_SOURCE_DIR})


    execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory "${DEBIAN_SOURCE_DIR}/debian")
    execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory "${DEBIAN_SOURCE_DIR}/debian/source")
    execute_process(COMMAND ${CMAKE_COMMAND} -E
                        copy ${CPACK_RESOURCE_FILE_LICENSE} ${DEBIAN_COPYRIGHT})

    file(WRITE "${DEBIAN_SOURCE_DIR}/debian/compat" "7")
    file(WRITE "${DEBIAN_SOURCE_DIR}/debian/source/format" "3.0 (native)")



    ##############################################################################
    # debian/control
    foreach(DEP ${CPACK_DEBIAN_BUILD_DEPENDS})
        set(${PPA_UBUNTU_NAME_BUILD_DEPENDS} "${PPA_UBUNTU_NAME_BUILD_DEPENDS}, ${DEP}")
    endforeach(DEP ${CPACK_DEBIAN_BUILD_DEPENDS})

    foreach(COMPONENT ${CPACK_COMPONENTS_ALL})
        set(PACKAGE ${CPACK_DEBIAN_PACKAGE_NAME}-${COMPONENT})
        if (PPA_UBUNTU_NAME_DEPENDS)
            set(${PPA_UBUNTU_NAME_DEPENDS} "${PACKAGE}")
        else()
            set(${PPA_UBUNTU_NAME_DEPENDS} ", ${PACKAGE}")
        endif()
    endforeach()

    file(WRITE ${DEBIAN_CONTROL}
        "Source: ${CPACK_DEBIAN_PACKAGE_NAME}\n"
        "Section: ${CPACK_DEBIAN_PACKAGE_SECTION}\n"
        "Priority: ${CPACK_DEBIAN_PACKAGE_PRIORITY}\n"
        "Maintainer: ${CPACK_PACKAGE_CONTACT}\n"
        "Build-Depends: ${CPACK_DEBIAN_BUILD_DEPENDS}\n"
        "Standards-Version: 3.9.7\n"
        "Homepage: ${CPACK_PACKAGE_VENDOR}\n")

    foreach(COMPONENT ${CPACK_COMPONENTS_ALL})
        string(TOUPPER ${COMPONENT} UPPER_COMPONENT)
        set(DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}")

        foreach(DEP ${CPACK_COMPONENT_${UPPER_COMPONENT}_DEPENDS})
            if (DEPENDS)
                set(DEPENDS "${DEPENDS}, ${CPACK_DEBIAN_PACKAGE_NAME}-${DEP}")
            else()
                set(DEPENDS "${CPACK_DEBIAN_PACKAGE_NAME}-${DEP}")
            endif()
        endforeach()

        foreach(DEP ${PPA_${COMPONENT}_DEPENDS})
            if (DEPENDS)
                set(DEPENDS "${DEPENDS}, ${DEP}")
            else()
                set(DEPENDS "${DEP}")
            endif()
        endforeach()

        file(APPEND ${DEBIAN_CONTROL}
            "\n"
            "Package: ${CPACK_DEBIAN_PACKAGE_NAME}-${COMPONENT}\n"
            "Architecture: any\n"
            "Depends: ${DEPENDS}\n"
            "Description: ${CPACK_PACKAGE_DESCRIPTION_SUMMARY}"
            ": ${CPACK_COMPONENT_${UPPER_COMPONENT}_DISPLAY_NAME}\n"
            "${DEB_LONG_DESCRIPTION}"
            " .\n"
            " ${CPACK_COMPONENT_${UPPER_COMPONENT}_DESCRIPTION}\n")
    endforeach(COMPONENT ${CPACK_COMPONENTS_ALL})



    ##############################################################################
    # debian/rules
    file(WRITE ${DEBIAN_RULES}
        "#!/usr/bin/make -f\n"
        "\n"
        #"BUILDDIR = build_dir\n"
        "\n"
        "build:\n"
        #"	mkdir -p $(BUILDDIR)\n"
        #"	cd $(BUILDDIR); cmake ..\n"
        #"	make -C $(BUILDDIR) preinstall\n"
        "	touch build\n"
        "\n"
        "binary: binary-indep binary-arch\n"
        "\n"
        "binary-indep: build\n"
        "\n"
        "binary-arch: build\n")

    foreach(COMPONENT ${CPACK_COMPONENTS_ALL})
        set(PATH debian/tmp_${COMPONENT})
        set(BUILDDIR build_${COMPONENT})
        set(PACKAGE ${CPACK_DEBIAN_PACKAGE_NAME}-${COMPONENT})
        file(APPEND ${DEBIAN_RULES}
            "	mkdir ${BUILDDIR}\n"
            "	cd ${BUILDDIR}; cmake -DCMAKE_INSTALL_PREFIX=../${PATH}/usr ${PPA_COMMON_CMAKE_ARGS} ${PPA_CMAKE_FLAGS_${COMPONENT}} ..\n"
            "	cd ${BUILDDIR}; make install\n"
            "	mkdir -p ${PATH}/DEBIAN\n"
            "	dpkg-gencontrol -p${PACKAGE} -P${PATH}\n"
            "	dpkg --build ${PATH} ..\n")
    endforeach(COMPONENT ${CPACK_COMPONENTS_ALL})

    file(APPEND ${DEBIAN_RULES}
        "\n"
        "clean:\n"
        "	rm -f build\n"
        "	rm -rf $(BUILDDIR)\n"
        "\n"
        ".PHONY: binary binary-arch binary-indep clean\n")

    execute_process(COMMAND chmod +x ${DEBIAN_RULES})


    ##############################################################################
    # debian/changelog
    execute_process(COMMAND date -R  OUTPUT_VARIABLE DATE_TIME)
    file(WRITE ${DEBIAN_CHANGELOG}
        "${CPACK_DEBIAN_PACKAGE_NAME} (${CPACK_PACKAGE_VERSION}~${UBUNTU_NAME}) ${UBUNTU_NAME}; urgency=low\n\n"
        "  * Package built with CMake\n")
    if(UBUNTU_LP_BUG)
        file(APPEND ${DEBIAN_CHANGELOG} "  * LP: #${UBUNTU_LP_BUG}\n")
    endif()
    file(APPEND ${DEBIAN_CHANGELOG} "\n -- ${CPACK_PACKAGE_CONTACT}  ${DATE_TIME}")


    ##############################################################################
    # build source package

    add_custom_command(
        OUTPUT ${DEBIAN_BASE_DIR}/${DEBIAN_SOURCE_CHANGES}
        COMMAND ${BUILDPACKAGE_EXECUTABLE} ${PPA_BUILDPACKAGE_FLAGS} -S
        WORKING_DIRECTORY ${DEBIAN_SOURCE_DIR}
        COMMENT "Generate ${CPACK_DEBIAN_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}")
    add_custom_target(${PPA_UBUNTU_NAME_TARGET}_changes DEPENDS "${DEBIAN_BASE_DIR}/${DEBIAN_SOURCE_CHANGES}")
    add_dependencies(${PPA_MAIN_TARGET} ${PPA_UBUNTU_NAME_TARGET}_changes)


    if(LINTIAN_EXECUTABLE)
        add_custom_command(
            TARGET ${PPA_UBUNTU_NAME_TARGET}_changes
            POST_BUILD
            COMMAND ${LINTIAN_EXECUTABLE} -cv
            WORKING_DIRECTORY ${DEBIAN_SOURCE_DIR}
            COMMENT "Checking package with lintian")
    endif()


    ##############################################################################
    # dput ppa:your-lp-id/ppa <source.changes>
    if(PPA_DPUT_HOST)
        add_custom_target(${PROJECT_NAME}_dput_${UBUNTU_NAME}
            ${DPUT_EXECUTABLE} ${PPA_DPUT_HOST} ${DEBIAN_SOURCE_CHANGES}
            DEPENDS ${DEBIAN_BASE_DIR}/${DEBIAN_SOURCE_CHANGES}
            WORKING_DIRECTORY ${DEBIAN_BASE_DIR}
            COMMENT "Upload ${CPACK_DEBIAN_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION} to ${PPA_DPUT_HOST}"
            DEPENDS ${PPA_UBUNTU_NAME_TARGET})
    endif()
endforeach()
