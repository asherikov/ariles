function(cmakeut_add_cpp_test TEST_NAME)
    #set(options OPTION0)
    #set(oneValueArgs OPTION1)
    set(multiValueArgs LIBS DEPENDS FLAGS)
    cmake_parse_arguments("CMAKEUT" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

    add_executable(${TEST_NAME} "./${TEST_NAME}.cpp")
    set_target_properties(${TEST_NAME} PROPERTIES COMPILE_FLAGS "${CMAKEUT_FLAGS}")

    target_link_libraries(${TEST_NAME} "${CMAKEUT_LIBS}")
    if (CMAKEUT_DEPENDS)
        add_dependencies(${TEST_NAME} "${CMAKEUT_DEPENDS}")
    endif()
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})

endfunction(cmakeut_add_cpp_test)
