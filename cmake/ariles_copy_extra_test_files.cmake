function(ariles_copy_extra_test_files TGT_MAIN TGT_THIS FROM_DIR PREFIX TO_DIR)
    set(COPY_FILES "")

    file(GLOB COPY_FILES_TMP "${FROM_DIR}/${PREFIX}*.ref")
    list(APPEND COPY_FILES ${COPY_FILES_TMP})

    file(GLOB COPY_FILES_TMP "${FROM_DIR}/${PREFIX}*.xml")
    list(APPEND COPY_FILES ${COPY_FILES_TMP})

    file(GLOB COPY_FILES_TMP "${FROM_DIR}/${PREFIX}*.json")
    list(APPEND COPY_FILES ${COPY_FILES_TMP})

    file(GLOB COPY_FILES_TMP "${FROM_DIR}/${PREFIX}*.yaml")
    list(APPEND COPY_FILES ${COPY_FILES_TMP})

    file(GLOB COPY_FILES_TMP "${FROM_DIR}/${PREFIX}*.m")
    list(APPEND COPY_FILES ${COPY_FILES_TMP})


    if (COPY_FILES)
        add_custom_target("${TGT_MAIN}_${TGT_THIS}")
        add_dependencies(${TGT_MAIN}   "${TGT_MAIN}_${TGT_THIS}")

        foreach(COPY_FILE ${COPY_FILES})
            add_custom_command( TARGET "${TGT_MAIN}_${TGT_THIS}" PRE_BUILD
                                COMMAND ${CMAKE_COMMAND} -E copy ${COPY_FILE} "${TO_DIR}")
        endforeach()
    endif()
endfunction()
