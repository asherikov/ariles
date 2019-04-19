# parameters
#   ARILES_MODULE               module
#   TEST_ID                     test id, number starting from 000
#   DEPENDENCIES                list of dependencies
#   [OPTIONAL_DEPENDENCIES]     list of optional dependencies, can be omitted
#
function(ariles_define_test ARILES_MODULE TEST_ID DEPENDENCIES)
    set(TEST_NAME        "${TEST_ID}")


    set(OPTIONAL_DEPENDENCIES   "${ARGV3}")
    set(LINK_TO_LIBRARIES       "")


    ariles_parse_test_dependencies("${DEPENDENCIES}" "${LINK_TO_LIBRARIES}" "${TGT_DEPENDS}")
    if (MISSING_DEPENDENCY)
        return()
    endif()
    ariles_parse_test_dependencies("${OPTIONAL_DEPENDENCIES}" "${LINK_TO_LIBRARIES}" "${TGT_DEPENDS}")


    set(TGT_NAME "${ARILES_MODULE}_${TEST_NAME}")

    add_executable(${TGT_NAME} "${TEST_NAME}.cpp")
    add_dependencies("${ARILES_MODULE}" "${TGT_NAME}")

    set_target_properties(${TGT_NAME} PROPERTIES OUTPUT_NAME "${TEST_NAME}")

    add_dependencies(${TGT_NAME} TGT_ariles_copy_headers ${TGT_DEPENDS})

    target_link_libraries(${TGT_NAME} ${LINK_TO_LIBRARIES})


    ariles_copy_extra_test_files(${TGT_NAME} "copy_cfg" "${CMAKE_CURRENT_LIST_DIR}" "${TEST_NAME}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
endfunction()
