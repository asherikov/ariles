function(ariles_detect_func_macro)
    include (CheckCXXSourceCompiles)

    CHECK_CXX_SOURCE_COMPILES("
        #include <iostream>
        int main(void)
        {
            std::cout << __func__ << std::endl;
            return 0;
        }"
        ARILES_COMPILER_SUPPORTS_FUNC_)

    CHECK_CXX_SOURCE_COMPILES("
        #include <iostream>
        int main(void)
        {
            std::cout << __FUNCTION__ << std::endl;
            return 0;
        }"
        ARILES_COMPILER_SUPPORTS_FUNCTION_)
endfunction(ariles_detect_func_macro)
