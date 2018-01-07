function(ariles_add_external_git_project)
    set(options NOBUILD)
    set(oneValueArgs TARGET_NAME PROJECT_DIR)
    set(multiValueArgs CMAKE_ARGS)
    cmake_parse_arguments("ARILES" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )


    if (EXISTS "${ARILES_PROJECT_DIR}/.git")
        message("Found '.git' subdirectory in project '${ARILES_PROJECT_DIR}': skipping download step.")

        if (ARILES_NOBUILD)
            ExternalProject_Add(
                ${ARILES_TARGET_NAME}
                DOWNLOAD_DIR    "${ARILES_PROJECT_DIR}"
                SOURCE_DIR      "${ARILES_PROJECT_DIR}"
                DOWNLOAD_COMMAND    ""
                BUILD_COMMAND       ""
                INSTALL_COMMAND     ""
                CONFIGURE_COMMAND   ""
                STEP_TARGETS    download
            )
        else (ARILES_NOBUILD)
            ExternalProject_Add(
                ${ARILES_TARGET_NAME}
                DOWNLOAD_DIR    "${ARILES_PROJECT_DIR}"
                SOURCE_DIR      "${ARILES_PROJECT_DIR}"
                CMAKE_ARGS      ${ARILES_CMAKE_ARGS}
                DOWNLOAD_COMMAND    ""
                STEP_TARGETS    download
            )
        endif (ARILES_NOBUILD)
    else()
        find_package(Git)

        if (GIT_FOUND)
            if (ARILES_NOBUILD)
                ExternalProject_Add(
                    ${ARILES_TARGET_NAME}
                    DOWNLOAD_DIR    "${ARILES_PROJECT_DIR}"
                    SOURCE_DIR      "${ARILES_PROJECT_DIR}"
                    DOWNLOAD_COMMAND    ${GIT_EXECUTABLE} submodule update --init --no-fetch ./
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    CONFIGURE_COMMAND   ""
                    STEP_TARGETS    download
                )
            else (ARILES_NOBUILD)
                ExternalProject_Add(
                    ${ARILES_TARGET_NAME}
                    DOWNLOAD_DIR    "${ARILES_PROJECT_DIR}"
                    SOURCE_DIR      "${ARILES_PROJECT_DIR}"
                    CMAKE_ARGS      ${ARILES_CMAKE_ARGS}
                    DOWNLOAD_COMMAND    ${GIT_EXECUTABLE} submodule update --init --no-fetch ./
                    STEP_TARGETS    download
                )
            endif (ARILES_NOBUILD)
        else(GIT_FOUND)
            message( SEND_ERROR "Cannot download project '${ARILES_PROJECT_DIR}' due to missing git. Checking if the files are alredy there..." )
            file(GLOB RESULT ${ARILES_PROJECT_DIR})
            list(LENGTH RESULT RES_LEN)
            if(RES_LEN EQUAL 0)
                message(FATAL_ERROR "Project directory '${ARILES_PROJECT_DIR}' is empty, compilation is impossible." )
            endif()
        endif(GIT_FOUND)
    endif()


    add_dependencies(${ARILES_TARGET_NAME}    TGT_fetch_bridges)
    add_dependencies(${ARILES_TARGET_FETCH_BRIDGES}    ${ARILES_TARGET_NAME}-download)
    set(ARILES_TARGET_FETCH_BRIDGES    "${ARILES_TARGET_NAME}-download"  CACHE INTERNAL "")
endfunction(ariles_add_external_git_project)
