function(cmakeut_copy_dir_if_exists TGT_MAIN TGT_THIS FROM_DIR TO_DIR)
    if (EXISTS "${FROM_DIR}")
        add_custom_target(  "${TGT_MAIN}_${TGT_THIS}"
                            COMMAND ${CMAKE_COMMAND} -E copy_directory "${FROM_DIR}" "${TO_DIR}")
        add_dependencies(${TGT_MAIN}   "${TGT_MAIN}_${TGT_THIS}")
    endif()
endfunction()
