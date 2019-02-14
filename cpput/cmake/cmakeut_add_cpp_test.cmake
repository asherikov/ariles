function(cmakeut_add_cpp_test TEST_NAME)

    #set(options OPTION0)
    #set(oneValueArgs OPTION1)
    set(multiValueArgs LIBS DEPENDS FLAGS)
    cmake_parse_arguments("CMAKEUT" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

    set(TGT_NAME "${PROJECT_NAME}_${TEST_NAME}")
    add_executable(${TGT_NAME} "./${TEST_NAME}.cpp")
    set_target_properties(${TGT_NAME} PROPERTIES COMPILE_FLAGS "${CMAKEUT_FLAGS}" OUTPUT_NAME ${TEST_NAME})

    target_link_libraries(${TGT_NAME} "${CMAKEUT_LIBS}")
    if (CMAKEUT_DEPENDS)
        add_dependencies(${TGT_NAME} "${CMAKEUT_DEPENDS}")
    endif()
    add_test(NAME ${TGT_NAME} COMMAND ${TEST_NAME})

endfunction(cmakeut_add_cpp_test)
