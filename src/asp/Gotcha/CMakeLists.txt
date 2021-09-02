# Use wrapper function at this level to avoid code duplication
add_library_wrapper(AspGotcha "${ASP_GOTCHA_SRC_FILES}" "${ASP_GOTCHA_TEST_FILES}" "${ASP_GOTCHA_LIB_DEPENDENCIES}")
