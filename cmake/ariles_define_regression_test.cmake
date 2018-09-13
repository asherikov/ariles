# parameters
#   ARILES_MODULE               module
#   REGRESSION_TEST_ID          test id, number starting from 000
#   DEPENDENCIES                list of dependencies
#   [OPTIONAL_DEPENDENCIES]     list of optional dependencies, can be omitted
#
function(ariles_define_regression_test ARILES_MODULE REGRESSION_TEST_ID DEPENDENCIES)
    set(TEST_NAME        "regression_test_${REGRESSION_TEST_ID}")


    set(OPTIONAL_DEPENDENCIES   "${ARGV3}")
    set(LINK_TO_LIBRARIES       "${ARILES_TESTING_LIBRARIES}")


    ariles_parse_test_dependencies("${DEPENDENCIES}" "${LINK_TO_LIBRARIES}" "${TGT_DEPENDS}")
    if (MISSING_DEPENDENCY)
        return()
    endif()
    ariles_parse_test_dependencies("${OPTIONAL_DEPENDENCIES}" "${LINK_TO_LIBRARIES}" "${TGT_DEPENDS}")

    list (FIND DEPENDENCIES "DIFF_WITH_REFERENCE" INDEX)
    if (${INDEX} GREATER -1)
        set(DIFF_WITH_REFERENCE ON)
    endif()


    set(TGT_NAME "${ARILES_MODULE}_${TEST_NAME}")

    add_executable(${TGT_NAME} "${TEST_NAME}.cpp")
    add_dependencies("${ARILES_MODULE}" "${TGT_NAME}")

    set_target_properties(${TGT_NAME} PROPERTIES OUTPUT_NAME "${TEST_NAME}")
    # TODO this is a workaround to suppress warnings due to UTF headers
    set_target_properties(${TGT_NAME} PROPERTIES COMPILE_FLAGS "-Wno-pedantic")

    add_dependencies(${TGT_NAME} TGT_ariles_copy_headers ${TGT_DEPENDS})

    target_link_libraries(${TGT_NAME} ${LINK_TO_LIBRARIES})
    target_link_libraries(${TGT_NAME} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARIES})
    target_link_libraries(${TGT_NAME} ${Boost_SYSTEM_LIBRARIES}             )
    target_link_libraries(${TGT_NAME} ${Boost_TIMER_LIBRARIES}              )
    target_link_libraries(${TGT_NAME} ${Boost_RANDOM_LIBRARIES}             )


    ariles_copy_extra_test_files(${TGT_NAME} "copy_cfg" "${CMAKE_CURRENT_LIST_DIR}" "${TEST_NAME}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

    if(DIFF_WITH_REFERENCE)
        set(REF_FILENAME "${CMAKE_CURRENT_LIST_DIR}/${TEST_NAME}.ref")
        add_test(NAME ${TGT_NAME} COMMAND sh -c "./${TEST_NAME} | grep -o --color=never \"^ariles.*\ =\\|^%.*\" | diff ${REF_FILENAME} -" WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    else()
        add_test(NAME ${TGT_NAME} COMMAND ${TEST_NAME} "${ARILES_CONFIG_DIR}/${ARILES_MODULE}/" "${TEST_NAME}.ref" WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    endif()
endfunction()
