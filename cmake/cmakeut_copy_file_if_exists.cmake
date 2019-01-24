function(cmakeut_copy_file_if_exists TGT_MAIN TGT_THIS COPY_FILE TO_DIR)
    if (EXISTS "${COPY_FILE}")
        add_custom_target("${TGT_MAIN}_${TGT_THIS}"
            COMMAND ${CMAKE_COMMAND} -E make_directory "${TO_DIR}"
            COMMAND ${CMAKE_COMMAND} -E copy "${COPY_FILE}" "${TO_DIR}")
        add_dependencies(${TGT_MAIN}   "${TGT_MAIN}_${TGT_THIS}")
    endif()
endfunction()
