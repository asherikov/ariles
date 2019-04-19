function(cmakeut_add_external_git_project)
    set(options NOBUILD)
    set(oneValueArgs TARGET_NAME PROJECT_DIR)
    set(multiValueArgs CMAKE_ARGS)
    cmake_parse_arguments("CMAKEUT" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )


    set (EXTERNAL_PROJECT_PARAMETERS
        ${CMAKEUT_TARGET_NAME}
        DOWNLOAD_DIR    "${CMAKEUT_PROJECT_DIR}"
        SOURCE_DIR      "${CMAKEUT_PROJECT_DIR}"
        STEP_TARGETS    download)


    if (CMAKEUT_NOBUILD)
        list(APPEND EXTERNAL_PROJECT_PARAMETERS
            BUILD_COMMAND       ""
            INSTALL_COMMAND     ""
            CONFIGURE_COMMAND   ""
        )
    else (CMAKEUT_NOBUILD)
        list(APPEND EXTERNAL_PROJECT_PARAMETERS
            CMAKE_ARGS      ${CMAKEUT_CMAKE_ARGS}
        )
    endif (CMAKEUT_NOBUILD)


    if (EXISTS "${CMAKEUT_PROJECT_DIR}/.git")
        message("Found '.git' subdirectory in project '${CMAKEUT_PROJECT_DIR}': skipping download step.")

        list(APPEND EXTERNAL_PROJECT_PARAMETERS
            DOWNLOAD_COMMAND    ""
        )
    else()
        find_package(Git)

        if (GIT_FOUND)
            list(APPEND EXTERNAL_PROJECT_PARAMETERS
                DOWNLOAD_COMMAND    ${GIT_EXECUTABLE} submodule update --init --no-fetch ./
            )
        else(GIT_FOUND)
            message( SEND_ERROR "Cannot download project '${CMAKEUT_PROJECT_DIR}' due to missing git. Checking if the files are alredy there..." )
            file(GLOB RESULT ${CMAKEUT_PROJECT_DIR})
            list(LENGTH RESULT RES_LEN)
            if(RES_LEN EQUAL 0)
                message(FATAL_ERROR "Project directory '${CMAKEUT_PROJECT_DIR}' is empty, compilation is impossible." )
            endif()
        endif(GIT_FOUND)
    endif()

    ExternalProject_Add(
        ${EXTERNAL_PROJECT_PARAMETERS}
    )
endfunction(cmakeut_add_external_git_project)
