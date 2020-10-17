# parameters
#   ARILES_MODULE               module
#   REGRESSION_TEST_ID          test id, number starting from 000
#   DEPENDENCIES                list of dependencies
#   [OPTIONAL_DEPENDENCIES]     list of optional dependencies, can be omitted
#
function(ariles_define_regression_test REGRESSION_TEST_ID DEPENDENCIES)
    set(TEST_NAME        "regression_test_${REGRESSION_TEST_ID}")


    set(OPTIONAL_DEPENDENCIES   "${ARGV2}")
    set(LINK_TO_LIBRARIES       "${ARILES_TESTING_LIBRARIES}")


    if ("${DEPENDENCIES}" STREQUAL "ANY")
        set(DEPENDENCIES "")
        set(DEPENDENCY_REQUIRED ON)
    endif()


    ariles_parse_test_dependencies("${DEPENDENCIES}" "${LINK_TO_LIBRARIES}" "${TGT_DEPENDS}" "${TGT_INCLUDES}")
    if (MISSING_DEPENDENCY)
        return()
    endif()
    ariles_parse_test_dependencies("${OPTIONAL_DEPENDENCIES}" "${LINK_TO_LIBRARIES}" "${TGT_DEPENDS}" "${TGT_INCLUDES}")
    if (MISSING_DEPENDENCY AND DEPENDENCY_REQUIRED)
        return()
    endif()

    list (FIND DEPENDENCIES "DIFF_WITH_REFERENCE" INDEX)
    if (${INDEX} GREATER -1)
        set(DIFF_WITH_REFERENCE ON)
    endif()


    set(TGT_NAME "${PROJECT_NAME}_${TEST_NAME}")

    add_executable(${TGT_NAME} "${TEST_NAME}.cpp")
    add_dependencies("${PROJECT_NAME}" "${TGT_NAME}")

    set_target_properties(${TGT_NAME} PROPERTIES OUTPUT_NAME "${TEST_NAME}")
    # TODO this is a workaround to suppress warnings due to Boost headers
    set_target_properties(${TGT_NAME} PROPERTIES COMPILE_FLAGS "-Wno-pedantic")

    if (TGT_INCLUDES)
        target_include_directories(${TGT_NAME} PRIVATE ${TGT_INCLUDES})
    endif()
    target_include_directories(${TGT_NAME} PRIVATE ${ARILES_CORE_BUILD_INCLUDES})
    target_include_directories(${TGT_NAME} SYSTEM PRIVATE ${ARILES_CORE_DEPENDENCY_INCLUDES})

    if (TGT_DEPENDS)
        add_dependencies(${TGT_NAME} ${TGT_DEPENDS})
    endif()

    target_link_libraries(${TGT_NAME} ${LINK_TO_LIBRARIES})
    target_link_libraries(${TGT_NAME} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARIES})
    target_link_libraries(${TGT_NAME} ${Boost_SYSTEM_LIBRARIES}             )
    target_link_libraries(${TGT_NAME} ${Boost_TIMER_LIBRARIES}              )
    target_link_libraries(${TGT_NAME} ${Boost_RANDOM_LIBRARIES}             )


    ariles_copy_extra_test_files(${TGT_NAME} "copy_cfg" "${CMAKE_CURRENT_LIST_DIR}" "${TEST_NAME}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

    add_test(NAME ${TGT_NAME} COMMAND ${TEST_NAME} WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

    set(TGT_TEST "${TGT_NAME}" PARENT_SCOPE)
endfunction()
