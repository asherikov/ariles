function(cmakeut_dump_variables)

    get_cmake_property(VARNAMES VARIABLES)

    foreach (VARNAME ${VARNAMES})
        message(STATUS "${VARNAME}=${${VARNAME}}")
    endforeach()

endfunction(cmakeut_dump_variables)
